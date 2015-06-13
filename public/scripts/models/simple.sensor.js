'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Simple sensor model that require a single value.
 */

var SimpleSensor = Backbone.Model.extend({

  defaults: {
    value: 0,
    min: 18,
    max: 25,
    name: 'SimpleSensor'
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;
    this.set({
      'dispatcherTopic': this.get('name').toLowerCase() + ':change'
    });

    /**
     * Notify other parts of the app for the voltage change.
     */

    this.on('change:value', function() {
      Dispatcher.trigger(_this.get('dispatcherTopic'), _this.attributes);
    });
  },

  /**
   * Listen for incoming data from the socket.io server.
   */

  subscribe: function(topic) {
    this.set({'topic': topic || this.get('topic')});
    console.log('Listening on topic ' + this.get('topic'));

    var _this = this;
    ioClient.on(this.get('topic'), function(msg) {
      _this.set({'value': msg.toFixed(2)});
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  },

  /**
   * Return the voltage as a percentage between min and max.
   */

  getPercentage: function() {
    var voltage = this.get('value');
    var min = this.get('min');
    var max = this.get('max');

    if (voltage > max) {
      return 100;
    } else if (voltage < min) {
      return 1;
    }

    var total = max - min;
    var diff = voltage - min;

    return ((diff/total) * 100).toFixed(1);
  }

});

module.exports = SimpleSensor;
