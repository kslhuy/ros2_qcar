const express = require('express');
const app = express();

// Serve static files
app.use(express.static('public'));

// Serve index page
app.get('/', (req, res) => {
    res.sendFile(__dirname + '/views/index.html');
});

// Add graceful shutdown
process.on('SIGINT', () => {
    console.log('\nReceived SIGINT. Graceful shutdown...');
    process.exit(0);
});

const PORT = 3000;
app.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
    console.log('Make sure ros2-web-bridge is running');
});