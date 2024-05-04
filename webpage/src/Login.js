import React, { useState, useEffect } from 'react';
import { Link, useNavigate } from 'react-router-dom';
import styled from 'styled-components';

const BACKEND_URL = process.env.REACT_APP_BACKEND_URL;

const Container = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-height: 100vh;
  background-color: #f0f2f5;
`;

const Form = styled.form`
  background: white;
  padding: 20px;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0,0,0,0.1);
  display: flex;
  flex-direction: column;
  gap: 15px;
  width: 100%;
  max-width: 300px;
`;

const Input = styled.input`
  padding: 10px;
  border: 1px solid #ccc;
  border-radius: 4px;
`;

const Button = styled.button`
  background-color: #007bff;
  color: white;
  padding: 10px 15px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.3s;

  &:hover {
    background-color: #0056b3;
  }
`;

const Message = styled.p`
  color: #cc0000;
`;

function LoginForm() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [msg, setMsg] = useState('');
  const [status, setStatus] = useState(null);
  const navigate = useNavigate();

  useEffect(() => {
    if (status === 'success') {
      navigate('/home');
    } else if (status === 'error') {
      setMsg("Invalid credentials");
    }
  }, [status, navigate]);

  const handleSubmit = (e) => {
    e.preventDefault();

    const userCredentials = { username, password };

    fetch(`${BACKEND_URL}/login`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(userCredentials)
    })
      .then(response => response.ok ? response.json() : Promise.reject('Invalid credentials'))
      .then(data => {
        localStorage.setItem('userId', data.userId);
        setStatus('success');
      })
      .catch(error => {
        console.error('Error:', error);
        setStatus('error');
      });
  };

  return (
    <Container>
      <h2>Login</h2>
      <Form onSubmit={handleSubmit}>
        <label htmlFor="username">Username:</label>
        <Input id="username" type="text" value={username} onChange={(e) => setUsername(e.target.value)} required />
        <label htmlFor="password">Password:</label>
        <Input id="password" type="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
        <Button type="submit">Login</Button>
      </Form>
      <Message>{msg}</Message>
      <p>
        Don't have an account? <Link to="/register">Create an account</Link>
      </p>
    </Container>
  );
}

export default LoginForm;
