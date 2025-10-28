// src/config.ts

/**
 * Dynamically gets the hostname (IP address or domain) from the browser's current URL.
 * This assumes the backend is running on the same device as the frontend is served from.
 * * Example: If you access the UI at http://192.168.1.50:3001, this will be "192.168.1.50".
 */
export const BACKEND_IP = window.location.hostname;

/**
 * Defines the port the backend server is running on.
 */
export const BACKEND_PORT = 5001; // Your backend port

/**
 * The full URL for the backend API.
 */
export const BACKEND_URL = `http://${BACKEND_IP}:${BACKEND_PORT}`;