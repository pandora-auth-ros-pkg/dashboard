'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var temperaturesTemplate = require('../templates/temperature.hbs');


var TemperatureView = Backbone.View.extend({

  el: $('#temperature'),

  template: temperaturesTemplate,

  initialize: function() {
    console.log('View temperature initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});


module.exports = TemperatureView;
