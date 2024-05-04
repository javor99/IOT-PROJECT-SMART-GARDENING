import React, { useState, useEffect } from 'react';
import styled from 'styled-components';

const Container = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 20px;
  max-width: 600px;
  margin: 0 auto;
  background-color: #f7f7f7;
`;

const Input = styled.input`
  padding: 10px;
  margin: 10px 0;
  border: 1px solid #ccc;
  border-radius: 4px;
  width: 100%;
`;

const Button = styled.button`
  padding: 10px 15px;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.3s;

  &:hover {
    background-color: #0056b3;
  }
`;

const List = styled.ul`
  list-style: none;
  padding: 0;
`;

const ListItem = styled.li`
  background: white;
  padding: 10px;
  margin-top: 10px;
  border-radius: 5px;
  display: flex;
  justify-content: space-between;
  align-items: center;
`;

const BACKEND_URL = process.env.REACT_APP_BACKEND_URL;

function DeviceManager() {
  const [deviceId, setDeviceId] = useState('');
  const [devices, setDevices] = useState([]);

  useEffect(() => {
    const userId = localStorage.getItem('userId');
    fetch(`${BACKEND_URL}/users/${userId}/devices`)
      .then(response => response.json())
      .then(data => setDevices(data))
      .catch(error => console.error('Error fetching devices:', error));
  }, []);

  const calculateDaysSinceCreated = (createdAt) => {
    const createdDate = new Date(createdAt);
    const currentDate = new Date();
    const timeDiff = currentDate - createdDate;
    return Math.floor(timeDiff / (1000 * 3600 * 24)); // Convert milliseconds to days
  };

  const handleConnect = () => {
    if (deviceId && !devices.some(device => device.deviceId === deviceId)) {
      const newDevice = { deviceId, humidity: '50', createdAt: new Date().toISOString() };
      const userId = localStorage.getItem('userId');

      fetch(`${BACKEND_URL}/users/${userId}/devices`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(newDevice),
      })
        .then(response => response.text())
        .then(data => {
          setDevices([...devices, {...newDevice, createdAt: new Date().toISOString()}]);
          setDeviceId('');
          console.log(`User ID: ${userId} added a new device: ${deviceId}`);
        })
        .catch(error => console.error('Error adding device:', error));
    }
  };

  const handleChangeHumidity = (deviceId) => {
    const newHumidity = prompt('Enter new humidity:', '50');
    if (newHumidity) {
      const userId = localStorage.getItem('userId');

      fetch(`${BACKEND_URL}/users/${userId}/devices/${deviceId}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ humidity: newHumidity }),
      })
        .then(response => response.text())
        .then(data => {
          console.log(`Humidity updated for device ${deviceId} in the database`);
          setDevices(devices.map(device =>
            device.deviceId === deviceId ? { ...device, humidity: newHumidity } : device
          ));
        })
        .catch(error => console.error('Error updating humidity:', error));
    }
  };

  const calculateBatteryPercentage = (createdAt) => {
    const createdDate = new Date(createdAt);
    const currentDate = new Date();
    const timeDiff = currentDate - createdDate; // Time difference in milliseconds
    const daysSinceCreated = Math.floor(timeDiff / (1000 * 3600 * 24)); // Convert to days

    const batteryPercentage = Math.max(0, ((28 - daysSinceCreated) / 28) * 100); // Battery remaining percentage
    return batteryPercentage.toFixed(2); // Return as a string with 2 decimal places
};

  return (
    <Container>
      <Input
        type="text"
        value={deviceId}
        onChange={(e) => setDeviceId(e.target.value)}
        placeholder="Enter Device ID"
      />
      <Button onClick={handleConnect}>Connect</Button>
      <List>
        {devices.map((device) => (
          <ListItem key={device.deviceId}>
            Device ID: {device.deviceId}, Humidity: {device.humidity},
            Battery: {device.createdAt ? `${calculateBatteryPercentage(device.createdAt)}%` : "100%"}
            <Button onClick={() => handleChangeHumidity(device.deviceId)}>Change Humidity</Button>
          </ListItem>
        ))}
      </List>
    </Container>
  );
}

export default DeviceManager;
