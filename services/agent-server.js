#!/usr/bin/env node

'use strict';

var zmq = require('zmq');
var io = require('socket.io-client');
var webSocket = require('ws');

/**
 * Env variables.
 */

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var ROS_BRIDGE_PORT = require('../utils/env').ROS_BRIDGE_PORT;
var LOCAL_IP = require('../utils/env').LOCAL_IP;

var killAgentService = '/gui/kill_agent';

/**
 * Connect with the socket.io server.
 */

var socket = io('http://localhost:3000');
var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

socket.on('connect', function() {
  console.log('Service agent controller connected to the socket.io server.');
});

ws.on('open', function() {
  console.log('Service agent controller connected to ROS Bridge.');
});

/**
 * Control the agent remotely.
 */

var agentHandler = zmq.socket('req');


socket.on('service/agent/command', function(cmd) {
  if (cmd === 'stop') {
    console.log('Stopping the agent.');

    var msg = {
      op: "call_service",
      service: killAgentService,
      args: [{}]
    };
    ws.send(JSON.stringify(msg));
    ws.on('message', function(msg) {
      console.log(msg);
      socket.emit('service/agent/status/success');
    });
  } else if (cmd === 'kill') {
    console.log('Killing the agent.');
    agentHandler.send(cmd);
  } else {
    console.log('Starting agent with strategy: ' + cmd);
    agentHandler.send(cmd);
  }
});

/**
 * Receive response from the agent handler.
 */

agentHandler.on('message', function(msg) {
  msg = msg.toString();
  if (msg === 'true') {
    console.log('Agent terminated successfully');
    socket.emit('service/agent/status/success');
  } else if (msg === 'false') {
    console.log('Agent didnt exit gracefully');
    socket.emit('service/agent/status/error');
  } else {
    console.log('Agent started with PID: ' + msg);
    socket.emit('service/agent/status/pid', msg);
  }
});

agentHandler.connect('tcp://' + ROS_MASTER_IP + ':5555');
