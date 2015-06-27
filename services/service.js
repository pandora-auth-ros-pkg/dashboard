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


var Service = function(options, onMessage) {
  this.name = options.name;
  this.topic = options.topic;
  this.op = options.op;
  this.msgType = options.msgType;
  this.serverTopic = options.serverTopic;
  this.onMessage = onMessage;
}

Service.prototype.start = function() {

  var _this = this;

  this.socket = io('http://localhost:' + SERVER_PORT);
  this.ws = new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

  this.client = {
    op: this.op,
    topic: this.topic,
    type: this.msgType
  };

  this.socket.on('connect', function(){
    console.log(_this.name + ' connected to the socket.io server.');
  });

  this.ws.on('open', function() {
    console.log(_this.name + ' connected to ROS bridge.');
    _this.ws.send(JSON.stringify(_this.client));
  });

  this.ws.on('message', function(data) {
    var data = JSON.parse(data.toString());
    console.log(data.msg);

    var filteredData = _this.onMessage(data.msg);
    _this.socket.emit('service/' + _this.serverTopic, filteredData);
  });
}

module.exports = Service;
