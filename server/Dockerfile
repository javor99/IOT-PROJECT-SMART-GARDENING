# Use an official Node.js runtime as the base image
FROM node:18

# Set the working directory inside the container
WORKDIR /app

# Copy package.json and package-lock.json to the container
COPY package*.json ./

# Install necessary dependencies directly inside the container
RUN npm install

# Copy the rest of the application files into the container
COPY . .

# Expose the port that the app will run on
EXPOSE 8080

# Set environment variables for the application
ENV DB_CONNECTION_URL="mongodb+srv://ivor:7Wf4tvFWUhab3tqg@cluster0.q2lhrqu.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"
ENV PORT=8080

# Start the application
CMD ["node", "index.js"]
