#!/usr/bin/env node

'use strict';

var sshClient = require('scp2');
var io = require('socket.io-client');
var webSocket = require('ws');

/**
 * Env variables.
 */

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var ROS_BRIDGE_PORT = require('../utils/env').ROS_BRIDGE_PORT;

/**
 * Connect with the socket.io server.
 */

var socket = io('http://localhost:3000');

socket.on('connect', function() {
  console.log('Service geotiff connected to the socket.io server.');
});

/**
 * Geotiff conventions.
 */

var fileName = 'TestMission';
var prefix = 'RRL_2015_PANDORA_';
var extension = '.tiff';

/**
 * Receive commands from the web client.
 */

socket.on('service/geotiff/request', function(filename) {

  fileName = filename;

  // Create Service request.
  var geotiffService = {
    "op": "call_service",
    "service": "/geotiff/saveMission",
    "args": [{
        data: fileName
    }]
  };

});

/**
 * Communicate with the geotiff service.
 */

var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

ws.on('open', function() {
  console.log('Connected to the ROS bridge.');
  ws.send(JSON.stringify(geotiffService));
});

ws.on('message', function(msg) {
  var response = JSON.parse(msg);
  console.log(msg);
  console.log('Received ' + response.result);

  console.log('Copying geotiff from the remote host.');
  sshClient.scp({
    host: ROS_MASTER_IP,
    username: 'pandora',
    password: 'pandora',
    path: '/home/pandora/' + prefix + fileName + extension
  }, '/tmp/', function(err) {
    if (err !== null) {
      console.log(err);
      socket.emit('service/geotiff/response', err);
    } else {
      console.log('Done.');
      socket.emit('service/geotiff/response', 'Geotiff saved!');
    }
  });
});
