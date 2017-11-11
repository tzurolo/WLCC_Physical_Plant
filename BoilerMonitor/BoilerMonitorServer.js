//
// Boiler Room Server
//
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
var net = require('net');
//var i2c = require('i2c');

var privateKey  = fs.readFileSync('/etc/letsencrypt/live/wlccfacilities.dyndns.org/privkey.pem');
var certificate = fs.readFileSync('/etc/letsencrypt/live/wlccfacilities.dyndns.org/fullchain.pem');
var certauth = fs.readFileSync('/etc/letsencrypt/live/wlccfacilities.dyndns.org/chain.pem');
var credentials = {
    key: privateKey,
    cert: certificate,
    ca: certauth
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
//  user authentication using Passport.js with a local strategy (username & password)
//
passport.use(new localStrategy(
  function(username, password, done) {
    // to start off, just hard-code username and pw check
    if ((username == 'facilities') && (password == '8606234912')) {
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
    user = { id: 1, username: 'facilities', password: '8606234912' };
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
}

turnLEDsOn();


//
// Data connection to camera program
//

var CAMDATAPORT = 3000;

var camDataServer = net.createServer(function(sock) {
    console.log('CONNECTED: ' + sock.remoteAddress +':'+ sock.remotePort);
    sock["mycounter"] = 0;
    // other stuff is the same from here
    // Add a 'data' event handler to this instance of socket
    sock.on('data', function(data) {
        sock.mycounter++;
        // console.log('DATA ' + sock.remoteAddress + ', count: ' + sock.mycounter);
        io.emit('tankData', data);
        
    });
    
    // Add a 'close' event handler to this instance of socket
    sock.on('close', function(data) {
        console.log('CLOSED: ' + sock.remoteAddress +' '+ sock.remotePort);
    });
    
});

camDataServer.listen(CAMDATAPORT, function() { //'listening' listener
  console.log('camera data server bound');
});

//
// End of data connection to camera program
//


//
//  child process for reading cam and opencv analysis
//
var child = execFile('/home/pi/BoilerMonitor/BoilerMonitorCV', [ CAMDATAPORT.toString() ]);
child.stdin.setEncoding('utf-8');

child.stdout.on('data', function (data) {
   // notify connected sockets of new data
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
app.get('/', isAuthenticated, function(req, res){
    res.sendFile(__dirname + '/index.html');
});
app.get('/home', isAuthenticated, function(req, res){
    res.sendFile(__dirname + '/home.html');
});
app.get('/status', isAuthenticated, function(req, res) {
    queryStatus(res);
});
app.use('/css', express.static('css'));
app.use('/images', isAuthenticated, express.static('images'));

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

console.log('Version: ' + process.version);

var port = 5051;
httpsServer.listen(port, function(){
  console.log('listening on *:' + port);
});
