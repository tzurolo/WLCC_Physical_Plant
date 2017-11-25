var express = require('express');
var app = express();
var http = require('http').Server(app);

var socketIo = require('socket.io');
var io = socketIo(http);

io.on('connection', function(socket){
    console.log('got a connection.');
  socket.on('command', function(msg){
      serialPort.write(msg + "\r");
      console.log('got command: ' + msg);
  });
  socket.on('disconnect', function () {
    console.log('lost connection.');
  });
});

console.log(__dirname);
app.use('/test.js', express.static(__dirname + '/test.js'));
app.use('/index', express.static(__dirname + '/index.html'));
//app.use('/', express.static(__dirname));
//app.get('/', function(req, res){
//  res.sendFile(__dirname + '/index.html');
//});
app.get('/', function(req, res) {
res.send('<h1>Hello</h1>');
});
http.listen(3000, function(){
console.log('listening on 3000');
});


