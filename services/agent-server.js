#!/usr/bin/env node

'use strict';

var zmq = require('zmq');
var io = require('socket.io-client');

/**
 * Env variables.
 */

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var LOCAL_IP = require('../utils/env').LOCAL_IP;
var SERVICE_NAME = 'victimAlert';
var validateVictimTopic = 'victim_goal';
var validateVictimTopicLength = Buffer.byteLength(validateVictimTopic);

/**
 * Connect with the socket.io server.
 */

var socket = io('http://localhost:3000');

socket.on('connect', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to the socket.io server.');
});

/**
 * Receive victim alerts from the agent, and validate them.
 */

var alertReceiver = zmq.socket('sub');
var validator = zmq.socket('pub');

/**
 * Control the agent remotely.
 */

var agentHandler = zmq.socket('req');


alertReceiver.subscribe(validateVictimTopic);

alertReceiver.on('message', function(data) {

  // Remove the topic name.
  var msg = data.toString('utf8', validateVictimTopicLength) .replace(/\s/g, '');
  msg = JSON.parse(msg);
  msg.probability = msg.probability.toFixed(2);
  msg.x = msg.x.toFixed(2);
  msg.y = msg.y.toFixed(2);
  console.log(msg);
  socket.emit('service/victim/alert', msg);
});

/**
 * Receive commands from the web client.
 */

socket.on('service/victim/response', function(res) {
  if (res === true) {
    console.log('the victim is valid.');
  } else {
    console.log('the victim is not valid');
  }
  validator.send(['victim_validation', res]);
});

socket.on('service/agent/command', function(cmd) {
  if (cmd === 'stop') {
    console.log('Stopping the agent.');
  } else {
    console.log('Starting agent with strategy: ' + cmd);
  }
  agentHandler.send(cmd);
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

alertReceiver.connect('tcp://' + ROS_MASTER_IP + ':6666');
agentHandler.connect('tcp://' + ROS_MASTER_IP + ':5555');

validator.bindSync('tcp://' + LOCAL_IP + ':6667');
