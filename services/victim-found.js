#!/usr/bin/env node

'use strict';

var zmq = require('zmq');

/**
 * Env variables
 */

var masterIP = require('../utils/env').getRosMasterIP();
var SERVICE_NAME = 'victimAlert';

/**
 * Connect with the socket.io server.
 */

var io = require('socket.io-client');
var socket = io('http://localhost:3000');

socket.on('connect', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to the socket.io server.');
});

/**
 * Set up subscribers and publishers.
 */

var alertReceiver = zmq.socket('sub');
var validator = zmq.socket('pub');

var validateVictimTopic = 'victim_goal';
var validateVictimTopicLength = Buffer.byteLength(validateVictimTopic);

alertReceiver.subscribe('victim_goal');

alertReceiver.on('message', function(data) {

  // Remove the topic name.
  var msg = data.toString('utf8', validateVictimTopicLength).replace(/\s/g, '');
  msg = JSON.parse(msg);
  msg.probability = msg.probability.toFixed(2);
  msg.x = msg.x.toFixed(2);
  msg.y = msg.y.toFixed(2);
  console.log(msg);
  socket.emit('service/victim/alert', msg);
});

socket.on('service/victim/response', function(res) {
  if (res === true) {
    console.log('the victim is valid.');
  } else {
    console.log('the victim is not valid');
  }
  validator.send(['victim_validation', res]);
});

alertReceiver.connect('tcp://' + masterIP + ':6666');
validator.bindSync('tcp://127.0.0.1:7777');
