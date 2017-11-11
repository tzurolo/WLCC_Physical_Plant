var net = require('net');

var HOST = 'localhost';
var PORT = 3000;

var server = net.createServer(function(sock) {
    console.log('CONNECTED: ' + sock.remoteAddress +':'+ sock.remotePort);
    sock["mycounter"] = 0;
    // other stuff is the same from here
    // Add a 'data' event handler to this instance of socket
    sock.on('data', function(data) {
        sock.mycounter++;
        console.log('DATA ' + sock.remoteAddress + ': ' + data + ', count: ' + sock.mycounter);
        // Write the data back to the socket, the client will receive it as data from the server
        sock.write('You said "' + data + '"');
        
    });
    
    // Add a 'close' event handler to this instance of socket
    sock.on('close', function(data) {
        console.log('CLOSED: ' + sock.remoteAddress +' '+ sock.remotePort);
    });
    
});

server.listen(8080, function() { //'listening' listener
  console.log('server bound');
   //console.log('Server listening on ' + server.address().address +':'+ server.address().port);
});
