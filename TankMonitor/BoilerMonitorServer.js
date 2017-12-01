//
// Boiler Room Server
//
var childProcess = require('child_process');
var Gpio = require('onoff').Gpio;
var exec = childProcess.exec;
var execFile = childProcess.execFile;

var express = require('express');
var app = express();

var http = require('http');
var socketIo = require('socket.io');
var net = require('net');
var httpServer = http.createServer(app);
var io = socketIo(httpServer);

//
// turn on LEDs
//
var LEDs = new Gpio(17, 'out');
LEDs.writeSync(1);

//
// Data connection to camera program
//

var CAMDATAPORT = 3000;

var camDataServer = net.createServer(function(sock) {
    console.log('CONNECTED from camera program: ' + sock.remoteAddress +':'+ sock.remotePort);
    sock.on('data', function(data) {
        io.emit('boilerData', data);
    });
    sock.on('close', function(data) {
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
//  routes
//
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});
app.get('/home', function(req, res){
    res.sendFile(__dirname + '/home.html');
});
app.get('/status', function(req, res) {
    queryStatus(res);
});
app.use('/css', express.static('css'));
app.use('/images', express.static('images'));

//
// end of routes
//

// web socket connection to web page
io.on('connection', function(socket){
    console.log('got a connection.');
  socket.on('command', function(msg){
      console.log('got command: ' + msg);
  });
  socket.on('disconnect', function () {
    console.log('lost connection.');
  });
});

console.log('Version: ' + process.version);

var port = 5051;
httpServer.listen(port, function(){
  console.log('listening on *:' + port);
});

//
// exit handler
//
function unexportOnClose() { //function to run when exiting program
    LEDs.writeSync(0); // Turn LED off
    LEDs.unexport(); // Unexport LED GPIO to free resources
    //client.destroy(); // kill client upon exit
    process.exit();
};
process.on('SIGINT', unexportOnClose);