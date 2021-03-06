'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var simpleSensorTemplate = require('../templates/simple.sensor.hbs');


var SimpleSensorView = Backbone.View.extend({

  el: $('.simple-sensors'),

  template: simpleSensorTemplate,

  initialize: function() {
    console.log('View simple sensor initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});


module.exports = SimpleSensorView;
