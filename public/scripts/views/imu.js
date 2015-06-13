'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var imuTemplate = require('../templates/imu.hbs');


var IMUView = Backbone.View.extend({

  el: $('#imu'),

  template: imuTemplate,

  initialize: function() {
    console.log('View IMU initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});


module.exports = IMUView;
