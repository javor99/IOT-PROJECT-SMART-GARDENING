// Import necessary modules
const express = require('express'); // Express framework for building web applications
const mongoose = require('mongoose'); // Mongoose for MongoDB interaction
const bcrypt = require('bcryptjs'); // Bcrypt for password hashing
const cors = require('cors'); // CORS middleware
const mqtt = require('mqtt'); // MQTT client
require('dotenv').config(); // Load environment variables

// Create an Express application
const app = express();

// Middleware setup
app.use(cors()); // Enable CORS for all routes
app.use(express.json()); // Middleware to parse JSON bodies

// MongoDB connection URL
const dbConnectionUrl = process.env.DB_CONNECTION_URL; // Use environment variables for security

// Define User schema using Mongoose
const userSchema = new mongoose.Schema({
  username: { type: String, required: true, unique: true },
  hashPassword: { type: String, required: true },
  devices: [{
    deviceId: String,
    humidity: String,
    createdAt: { type: Date, default: Date.now } // Automatically set to current date and time
  }]
});

// MQTT connection options
const mqttOptions = {
  host: 'test.mosquitto.org',
  port: 1883,
  protocol: 'tcp'
};

// Connect to MQTT broker
const client = mqtt.connect(mqttOptions);

// Create a Mongoose model for User
const User = mongoose.model('User', userSchema);

// Connect to MongoDB
mongoose.connect(dbConnectionUrl, { useNewUrlParser: true, useUnifiedTopology: true })
  .then(() => console.log('MongoDB connected'))
  .catch(err => console.error('MongoDB connection error:', err));

// Signup endpoint
app.post('/signup', async (req, res) => {
  try {
    const { username, password } = req.body;
    if (!username || !password) return res.status(400).send('Username and password are required.');

    const existingUser = await User.findOne({ username });
    if (existingUser) return res.status(500).send('User already exists.');

    const hashPassword = await bcrypt.hash(password, 10); // Salt rounds set to 10
    const newUser = new User({ username, hashPassword });
    await newUser.save();

    res.status(201).json({ userId: newUser._id, message: 'User created successfully.' });
  } catch (error) {
    res.status(500).send('Error creating user.');
  }
});

// Login endpoint
app.post('/login', async (req, res) => {
  try {
    const { username, password } = req.body;
    if (!username || !password) return res.status(400).send('Username and password are required.');

    const user = await User.findOne({ username });
    if (!user) return res.status(404).send('User not found.');

    const isMatch = await bcrypt.compare(password, user.hashPassword);
    if (!isMatch) return res.status(401).send('Invalid credentials.');

    res.json({ userId: user._id, message: 'Login successful.' });
  } catch (error) {
    res.status(500).send('Error logging in.');
  }
});

// Endpoint to get all users
app.get('/users', async (req, res) => {
  try {
    const users = await User.find();
    res.send(users);
  } catch (error) {
    res.status(500).send('Error getting users.');
  }
});

// Endpoint to get a user by ID
app.get('/users/:userId', async (req, res) => {
  try {
    const { userId } = req.params;
    const user = await User.findById(userId);
    if (!user) return res.status(404).send('User not found.');
    res.send(user);
  } catch (error) {
    res.status(500).send('Error getting user.');
  }
});

// Endpoint to get all devices for a user
app.get('/users/:userId/devices', async (req, res) => {
  try {
    const { userId } = req.params;
    const user = await User.findById(userId);
    if (!user) return res.status(404).send('User not found.');
    res.send(user.devices);
  } catch (error) {
    res.status(500).send('Error getting devices.');
  }
});

// Endpoint to create a new device for a user
app.post('/users/:userId/devices', async (req, res) => {
  try {
    const { userId } = req.params;
    const { deviceId, humidity } = req.body;

    const user = await User.findById(userId);
    if (!user) return res.status(404).send('User not found.');

    const existingDevice = user.devices.find(device => device.deviceId === deviceId);
    if (existingDevice) return res.status(409).send('Device already exists.');

    const newDevice = {
      deviceId,
      humidity,
      createdAt: new Date() // Assigning the current date and time
    };

    user.devices.push(newDevice);
    await user.save();

    client.publish('project/newDevice', `id:${deviceId}`, (err) => {
      if (err) {
        console.error(`Failed to publish new device ID ${deviceId}`);
      } else {
        console.log(`New device ID ${deviceId} published successfully on MQTT`);
      }
    });

    res.status(201).send('Device created successfully.');
  } catch (error) {
    res.status(500).send('Error creating device.');
  }
});

// Endpoint to update device humidity for a user
app.put('/users/:userId/devices/:deviceId', async (req, res) => {
  try {
    const { userId, deviceId } = req.params;
    const { humidity } = req.body;

    const user = await User.findById(userId);
    if (!user) return res.status(404).send('User not found.');

    const device = user.devices.find(device => device.deviceId === deviceId);
    if (!device) return res.status(404).send('Device not found.');

    device.humidity = humidity;
    await user.save();

    client.publish(`project/newHumidity`, `${deviceId}:${humidity}`, (err) => {
      if (err) {
        console.error(`Failed to publish humidity for device ${deviceId}`);
      } else {
        console.log(`Humidity for device ${deviceId} published successfully`);
      }
    });

    res.send('Device humidity updated successfully.');
  } catch (error) {
    res.status(500).send('Error updating device humidity.');
  }
});

// Endpoint to publish humidity for a device
app.post('/publish-humidity', (req, res) => {
  const { deviceId, humidity } = req.body;

  client.publish(`project/newHumidity`, `${deviceId}:${humidity}`, (err) => {
    if (err) {
      console.error(`Failed to publish humidity for device ${deviceId}`);
      res.status(500).json({ error: 'Failed to publish humidity' });
    } else {
      console.log(`Humidity for device ${deviceId} published successfully`);
      res.status(200).json({ message: 'Humidity published successfully' });
    }
  });
});

// Start the server
const PORT = process.env.PORT || 8080; // Use environment variable for port or default to 8080
app.listen(PORT, () => console.log(`Server running on port ${PORT}`));
