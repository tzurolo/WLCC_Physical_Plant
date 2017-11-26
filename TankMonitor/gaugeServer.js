var childProcess = require('child_process');
var exec = childProcess.exec;
var execFile = childProcess.execFile;

var fs = require('fs');
var i2c = require('i2c');

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
function turnLEDsOn () {
    exec('gpio mode 0 out && gpio write 0 1');
}

turnLEDsOn();

//
//  child process for reading cam and opencv analysis
//
var child = execFile('/home/pi/TankMonitor/GaugeReader');
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
