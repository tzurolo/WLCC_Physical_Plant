var net = require('net');

var server = net.createServer(function(socket) {
    console.log('got a client');
    socket.write('Echo server\r\n');
    socket.on('data', function(data) {
        
        console.log('DATA ' + sock.remoteAddress + ': ' + data);
        // Write the data back to the socket, the client will receive it as data from the server
        sock.write('You said "' + data + '"');
    });
    socket.on('close', function(data) {
        console.log('lost client');
    });
});

server.listen(3001, '127.0.0.1');