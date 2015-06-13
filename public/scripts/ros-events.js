'use strict';

var io = require('socket.io-client');

var socket = io();

socket.on('connect', function() {
  console.log('Client conneccted with the socket.io server.');
});

module.exports = socket;
