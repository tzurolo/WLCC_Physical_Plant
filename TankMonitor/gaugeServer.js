var childProcess = require('child_process');
var exec = childProcess.exec;
var execFile = childProcess.execFile;

var fs = require('fs');
var express = require('express');
var app = express();
var passport = require('passport');
var localStrategy = require('passport-local').Strategy;

//var http = require('http').Server(app);
var https = require('https');
var socketIo = require('socket.io');
var i2c = require('i2c');

var privateKey  = fs.readFileSync('ssl/myPrivateKey.key');
var certificate = fs.readFileSync('ssl/myCert.crt');
var credentials = {
    key: privateKey,
    cert: certificate
};
var httpsServer = https.createServer(credentials, app);
var io = socketIo(httpsServer);

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
//	SQLite
//
var insertDevEventStmt;
var insertTempDataStmt;
var insertImageFileStmt;
var deleteImageFilesStmt;

var sqlite3 = require('sqlite3').verbose();
var db = new sqlite3.Database('tankdata.sqlite');
 
db.serialize(function() {
  db.run("CREATE TABLE IF NOT EXISTS tankData (timestamp TEXT, level TEXT)");
  db.run("CREATE TABLE IF NOT EXISTS tempData (timestamp TEXT, temp REAL)");
  db.run("CREATE TABLE IF NOT EXISTS imageFiles (timestamp TEXT, filename TEXT)");
 
  insertTankDataStmt = db.prepare("INSERT INTO tankData VALUES (?, ?)");
  insertTempDataStmt = db.prepare("INSERT INTO tempData VALUES (?, ?)");
  insertImageFileStmt = db.prepare("INSERT INTO imageFiles VALUES (?, ?)");
  deleteImageFilesStmt = db.prepare("DELETE FROM imageFiles WHERE timestamp < ?");
 
});

function queryTankHistory (numDays, res) {
    // find latest event for device
    var timestampConstraint = new Date();
    timestampConstraint.setDate(timestampConstraint.getDate() - numDays);
    var timestampParam = timestampConstraint.toISOString();
    db.all("SELECT timestamp,level FROM tankEvents " +
       " WHERE timestamp>? " +
       " ORDER BY timestamp desc LIMIT 100",
       timestampParam,
    function(err, rows) {
        if (!err) {
            res.json(rows);
        } else {
            res.send('error getting tank data');
        }
    });
 }

function purgeCameraImages (camImageRetentionMinutes) {
    // find camera images beyond the retention period
    var timestampConstraint = new Date();
    timestampConstraint.setMinutes(
        timestampConstraint.getMinutes() - camImageRetentionMinutes);
    var timestampParam = timestampConstraint.toISOString();
    console.log('looking for image files older than ' + timestampParam);
    // find the files that need to be deleted
    db.all("SELECT filename FROM imageFiles " +
        " WHERE timestamp<?",
        timestampParam,
    function(err, rows) {
        if (!err) {
            if (rows.length > 0) {
                // delete the files
                for (i = 0; i < rows.length; ++i) {
                    var fileToDelete = rows[i].filename;
                    console.log('deleting file ' + fileToDelete);
                    fs.unlink(fileToDelete, function(err) {
                        if (err) {
                            return console.error(err);
                        }
                    });
                }

                // delete the rows from the database
                deleteImageFilesStmt.run(timestampParam);
                deleteImageFilesStmt.reset();
            }
        } else {
            console.log('error getting image files');
        }
    });
 }

//
//	end of SQLite
//

//
//  user authentication using Passport.js with a local strategy (username & password)
//
passport.use(new localStrategy(
  function(username, password, done) {
    // to start off, just hard-code username and pw check
    if ((username == 'tonyz') && (password == 'adafruit')) {
        console.log('login succeeded for "' + username + '" at ' + Date());
        user = { id: 1, username: username, password: password };
        return done(null, user);
    } else {
        console.log('login failed for "' + username + '" at ' + Date());
        return done(null, false);
    }
  }));

passport.serializeUser(function(user, done) {
  done(null, user.id);
});

passport.deserializeUser(function(id, done) {
    user = { id: 1, username: 'tonyz', password: 'adafruit' };
    done(null, user);
});

app.use(require('cookie-parser')());
app.use(require('body-parser').urlencoded({ extended: true }));
app.use(require('express-session')({ secret: 'there is one mediator', resave: false, saveUninitialized: false }));
app.use(passport.initialize());
app.use(passport.session());

function isAuthenticated(req, res, next) {
    if (req.user)
        return next();
    else
        res.redirect('/login');
}

//
//  end of user authentication
//

//
// turn on LEDs
//
function turnLEDsOn () {
    exec('gpio mode 0 out && gpio write 0 1');
    exec('gpio mode 1 out && gpio write 1 1');
}

turnLEDsOn();

//
//  child process for reading cam and opencv analysis
//
var child = execFile('/home/pi/gaugeMonitor/GaugeReader');
child.stdin.setEncoding('utf-8');

child.stdout.on('data', function (data) {
   // notify connected sockets of new data
   var readerJSON = '{ "gauge": ' + data +
   ', "temp": "' + convertTempToF(latestTempC) + '"}';
   console.log('reader JSON: "' + readerJSON + '"');
   io.emit('tankData', readerJSON);

    // put new data in database
//    var gaugeData = JSON.parse(data);
//    insertTankDataStmt.run(gaugeData.timestamp, gaugeData.level);
//    insertTankDataStmt.reset();
//    insertImageFileStmt.run(gaugeData.timestamp, gaugeData.imageFile);
//    insertImageFileStmt.reset();

    // purge away old image files
    purgeCameraImages(settings.camImageRetention);
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
app.get('/', isAuthenticated, function(req, res){
    res.sendFile(__dirname + '/index.html');
});
app.get('/home', isAuthenticated, function(req, res){
    res.sendFile(__dirname + '/home.html');
});
app.get('/home.html', isAuthenticated, function(req, res){  // keeping this until we update all bookmarks
    res.sendFile(__dirname + '/home.html');
});
app.get('/status', isAuthenticated, function(req, res) {
    queryStatus(res);
});
app.post('/history', isAuthenticated, function(req, res) {
    var device = req.body.device;
    var numDays = req.body.numDays;
    queryDeviceHistory(device, numDays, res);
});
app.use('/css', express.static('css'));
app.use('/images', isAuthenticated, express.static('images'));
app.use('/camImages', isAuthenticated, express.static('camImages'));

app.get('/login',
  function(req, res){
      res.sendFile(__dirname + '/login.html');
  });
  
app.post('/login', 
  passport.authenticate('local', { successReturnToOrRedirect: '/home',
                                             failureRedirect: '/login' }));

app.get('/logout',
  function(req, res){
    if (req.user) {
        username = req.user.username;
    } else {
        username = '?';
    }
    console.log('user "' + username + '" is logging off at ' + Date());
    req.logout();
    res.redirect('/login');
  });

//
// end of routes
//

io.on('connection', function(socket){
    console.log('got a connection.');
  socket.on('command', function(msg){
//      sendCommandToMicrocontroller(msg);
      console.log('got command: ' + msg);
  });
  socket.on('disconnect', function () {
    console.log('lost connection.');
  });
});

function readGauge () {
    child.stdin.write('readCam\n');
}
readGauge();
setInterval(function(){ readGauge(); }, minutesToMilliseconds(settings.tankSampleTime));

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

        // store temp data in database
        var now = new Date();
        insertTempDataStmt.run(now.toISOString(), latestTempC);
        insertTempDataStmt.reset();
    });
}
readTemp();
setInterval(function(){ readTemp(); }, minutesToMilliseconds(settings.tempSampleTime));
// 
// end of tmp102 temperature sensor
//

console.log('Version: ' + process.version);

var port = 5801;
httpsServer.listen(port, function(){
  console.log('listening on *:' + port);
});
