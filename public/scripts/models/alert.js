'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');


var Alert = Backbone.Model.extend({

  defaults: {
    name: 'Generic Alert'
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;
  },

  /**
   * Listen for incoming alerts from the socket.io server.
   */

  subscribe: function(topic) {
    this.set({'topic': topic || this.get('topic')});
    console.log('Listening on topic ' + this.get('topic'));

    var _this = this;
    ioClient.on(this.get('topic'), function(msg) {
      _this.set({'info': msg});
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  }
});

module.exports = Alert;
