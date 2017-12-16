//
// Oil Tank Gauge Server
//
var net = require('net');
var Gpio = require('onoff').Gpio;
var childProcess = require('child_process');
var exec = childProcess.exec;
var execFile = childProcess.execFile;
var fs = require('fs');
var i2c = require('i2c');

// these packages are used to provide web access to this server
// This is only used during calibration
var http = require('http');
var express = require('express');
var app = express();
var httpServer = http.createServer(app);
var socketIo = require('socket.io')(httpServer);
var net = require('net');

// convert temperature from Centigrade to Fahrenheit
function convertTempToF (
    degC)
{
    var degF = ((parseFloat(degC) * 9.0) / 5.0) + 32.0;
    return degF.toFixed(1) + '&#x2109';
}

// times are in minutes
var settings = {
   "tankSampleTime": "0.08",
   "tempSampleTime": "5",
   "camImageRetention": "5"
};
function minutesToMilliseconds (minutes) {
  return minutes * 60000;
}

//
// turn on LEDs
//
var LEDs = new Gpio(17, 'out');
LEDs.writeSync(1);

//
//  Connection to Boiler Room server
//
var boilerRoomServerConnected = false;
var connectRetryTimer = null;
var client = null;

function newSocket () {
client = new net.Socket();
client.connect(3001, '192.168.1.71', function() {
    boilerRoomServerConnected = true;
    if (connectRetryTimer !== null) {
        clearInterval(connectRetryTimer);
        connectRetryTimer = null;
    }
    console.log('Connected to boiler room server');
});

client.on('error', function(e) {
    console.log('error: ' + e);
});

client.on('data', function(data) {
    console.log('Received: ' + data);
});

client.on('close', function() {
    boilerRoomServerConnected = false;
    if (connectRetryTimer == null) {
        connectRetryTimer = setInterval(function(){ newSocket(); }, 15000);
    }
    console.log('Connection to boiler room server closed');
});
}
newSocket();
//
//  end of Connection to Boiler Room server
//

//
// Data connection to camera program
//

var CAMDATAPORT = 3000;
var camDataSocket = null;

var camDataServer = net.createServer(function(sock) {
    console.log('CONNECTED from camera program: ' + sock.remoteAddress +':'+ sock.remotePort);
    camDataSocket = sock;
    sock.on('data', function(data) {
        if (boilerRoomServerConnected) {
            client.write(data);
        }
        //socketIo.emit('boilerData', data);
    });
    sock.on('close', function(data) {
        camDataSocket = null;
        console.log('camera program connection CLOSED: ' + sock.remoteAddress +' '+ sock.remotePort);
    });
    
});
camDataServer.listen(CAMDATAPORT, function() { //'listening' listener
  console.log('tank data server bound');
});

//
// End of data connection to camera program
//

//
//  child process for reading cam and opencv analysis
//
var child = execFile('/home/pi/TankMonitor/GaugeReader', [ CAMDATAPORT.toString() ]);
child.stdin.setEncoding('utf-8');

child.stdout.on('data', function (data) {
   console.log('child stdout: "' + data + '"');
});

child.stderr.on('data', function (data) {
   console.log('child stderr: "' + data + '"');
});

//
//  end of child process for reading cam and opencv analysis
//

// 
// tmp102 temperature sensor
//
var tmp102Address = 0x48;
var tmp102 = new i2c(tmp102Address, {device: '/dev/i2c-1'});
var latestTempC = 0;
function readTemp () {
    // set tmp102 pointer to 0
    tmp102.writeByte(0, function(err) {});
    // read tmp102 temperature register
    tmp102.read(2, function(err, res) {
        // compute temperature from sensor data
        var rawSensorData = res[0] * 256 + res[1];
        var tempSixteenthsC = rawSensorData / 16;
        latestTempC = tempSixteenthsC / 16;

        var now = new Date();
        console.log('temp: ' + latestTempC + ' at ' + now.toISOString());
    });
}
readTemp();
setInterval(function(){ readTemp(); }, minutesToMilliseconds(settings.tempSampleTime));
// 
// end of tmp102 temperature sensor
//

console.log('Version: ' + process.version);

//
//  routes
//
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});
app.get('/RaspiCamViewer', function(req, res){
    res.sendFile(__dirname + '/RaspiCamViewer.html');
});
app.get('/status', function(req, res) {
    queryStatus(res);
});
app.use('/css', express.static('css'));
app.use('/images', express.static('images'));

//
// end of routes
//

//
// web socket connection to web page
//
socketIo.on('connection', function(socket){
    console.log('got a connection.');
  socket.on('command', function(cmd){
      //console.log('got command: ' + cmd);
      if (camDataSocket !== null) {
        camDataSocket.write(cmd + '\r');
      }
  });
  socket.on('disconnect', function () {
    console.log('lost connection.');
  });
});
//
// end of web socket connection to web page
//

var port = 5052;
httpServer.listen(port, function(){
  console.log('listening on *:' + port);
});

//
// exit handler
//
function unexportOnClose() { //function to run when exiting program
    LEDs.writeSync(0); // Turn LED off
    LEDs.unexport(); // Unexport LED GPIO to free resources
    client.destroy(); // kill client upon exit
    process.exit();
};
process.on('SIGINT', unexportOnClose);