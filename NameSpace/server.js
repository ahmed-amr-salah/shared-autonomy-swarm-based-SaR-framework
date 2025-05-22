const http = require('http');
const fs = require('fs');
const path = require('path');
const socketio = require('socket.io');

const hostname = '172.17.0.1';
const port = 8000;

const server = http.createServer((req, res) => {
  const filePath = path.join(__dirname, 'dashboardkelaby.html');

  fs.readFile(filePath, 'utf8', (err, htmlContent) => {
    if (err) {
      res.writeHead(500, { 'Content-Type': 'text/plain' });
      res.end('Internal Server Error');
    } else {
      res.writeHead(200, { 'Content-Type': 'text/html' });
      res.end(htmlContent);
    }
  });
});

const io = socketio(server, {
    cors: {
      origin: '*',
      methods: ['GET', 'POST']
    }
  });

const moveNamespace = io.of('/move'); // Define the '/move' namespace

moveNamespace.on('connection', (socket) => {
  console.log('A client connected to the /move namespace');
  // Add your Socket.IO event listeners and handlers specific to the '/move' namespace here
});

server.listen(port, hostname, () => {
  console.log(`Server running at http://${hostname}:${port}/`);
});
