#!/usr/bin/env node


'use strict';

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var ROS_BRIDGE_PORT = require('../utils/env').ROS_BRIDGE_PORT;
var SERVER_PORT = require('../utils/env').SERVER_PORT;


var io = require('socket.io-client');
var webSocket = require('ws');

var socket = io('http://localhost:' + SERVER_PORT);
var ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);


socket.on('connect', function() {
  console.log('Service victim info connected to the socket.io server.');
});

ws.on('open', function() {
  console.log('Service victim info connected to the ROS bridge server.');
});

socket.on('service/victimProbabilities/get', function(victimID) {
  console.log('Sending message');
  victimID = parseInt(victimID);
  var msg = {
    op: 'call_service',
    service: '/data_fusion/get_probabilities',
    args: [
      victimID
    ]
  };

  ws.send(JSON.stringify(msg));
});

ws.on('message', function(data) {
  data = JSON.parse(data);
  console.log(data.values.success);

  if (data.values.success == true) {
    var info = [
      {
        name: "sound",
        value: (data.values.probabilities.sound * 100).toFixed(2)
      }, {
        name: "visualVictim",
        value: (data.values.probabilities.visualVictim * 100).toFixed(2)
      }, {
        name: "co2",
        value: (data.values.probabilities.co2 * 100).toFixed(2)
      }, {
        name: "thermal",
        value: (data.values.probabilities.thermal * 100).toFixed(2)
      }, {
        name: "hazmat",
        value: (data.values.probabilities.hazmat * 100).toFixed(2)
      }, {
        name: "motion",
        value: (data.values.probabilities.motion * 100).toFixed(2)
      }
    ];
    socket.emit('service/victimProbabilities/response', info);
    console.log(info);
  } else {
    socket.emit('service/victimProbabilities/error');
  }
});
