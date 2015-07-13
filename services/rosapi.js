#!/usr/bin/env node

'use strict';

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var ROS_BRIDGE_PORT = require('../utils/env').ROS_BRIDGE_PORT;
var SERVER_PORT = require('../utils/env').SERVER_PORT;

/**
 * Load dependencies.
 */

var io = require('socket.io-client');
var webSocket = require('ws');
var socket = io('http://localhost:' + SERVER_PORT);


socket.on('connect', function() {
  console.log('Service rosapi connected to the socket.io server.');
});

socket.on('service/rosapi/request', function(req) {
  var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

  ws.on('open', function() {
    console.log('Service rosapi connected to ROS bridge.');
    console.log('Request: ' + req);
    var msg = {
      op: 'call_service',
      service: '/rosapi/' + req,
      args: []
    };
    ws.send(JSON.stringify(msg));
  });

  ws.on('message', function(data) {
    data = JSON.parse(data.toString());
    console.log(data);

    socket.emit('service' + data.service, data.values);
  });
});

socket.on('service/rosapi/subscribers', function(req) {
  var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

  ws.on('open', function() {
    console.log('Service rosapi connected to ROS bridge.');
    console.log('Request: ' + req);
    var msg = {
      op: 'call_service',
      service: '/rosapi/subscribers',
      args: [req]
    };
    ws.send(JSON.stringify(msg));
  });

  ws.on('message', function(data) {
    data = JSON.parse(data.toString());
    console.log(data);

    socket.emit('service/rosapi/subscribers/response', data.values);
  });
});

socket.on('service/rosapi/publishers', function(req) {
  var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

  ws.on('open', function() {
    console.log('Service rosapi connected to ROS bridge.');
    console.log('Request: ' + req);
    var msg = {
      op: 'call_service',
      service: '/rosapi/publishers',
      args: [req]
    };
    ws.send(JSON.stringify(msg));
  });

  ws.on('message', function(data) {
    data = JSON.parse(data.toString());
    console.log(data);

    socket.emit('service/rosapi/publishers/response', data.values);
  });
});
