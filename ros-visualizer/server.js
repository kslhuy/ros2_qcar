const express = require('express');
const app = express();

// Serve static files
app.use(express.static('public'));

// Serve main page
app.get('/', (req, res) => {
    res.sendFile(__dirname + '/views/index.html');
});

// Serve settings page
app.get('/settings', (req, res) => {
    res.sendFile(__dirname + '/views/settings.html');
});

// Add graceful shutdown
process.on('SIGINT', () => {
    console.log('\nReceived SIGINT. Graceful shutdown...');
    process.exit(0);
});

const PORT = 3000;
app.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
    console.log(`Settings page: http://localhost:${PORT}/settings`);
    console.log('Make sure ros2-web-bridge is running');
});