'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Model for the Battery indicator.
 *
 * @param name String - name of the battery.
 * @param voltage Integer - the output of the battery.
 * @param percentage Float - the voltage as a percentage between min and max.
 * @param min Integer - the minumum value the voltage can take.
 * @param max Integer - the maximum value the voltage can take.
 * @param topic String - socket.io topic to listen to for incoming data.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var Battery = Backbone.Model.extend({

  defaults: {
    voltage: 0,
    min: 18,
    max: 25,
    name: 'Battery'
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;
    this.set({
      'dispatcherTopic': 'battery:' + this.get('name').toLowerCase() + ':change'
    });

    /**
     * Notify other parts of the app for the voltage change.
     */

    this.on('change:voltage', function() {
      _this.set({'percentage': _this.getPercentage()});
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
      _this.set({'voltage': msg.voltage.toFixed(2)});
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  },

  /**
   * Return the voltage as a percentage between min and max.
   */

  getPercentage: function() {
    var voltage = this.get('voltage');
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

module.exports = Battery;
