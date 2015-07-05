'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var batteryNormal = require('../templates/battery.normal.hbs');
var batteryDanger = require('../templates/battery.danger.hbs');


var BatteryView = Backbone.View.extend({

  el: $('#batteries'),

  templateNormal: batteryNormal,
  templateDanger: batteryDanger,

  initialize: function() {
    console.log('View Battery initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    if (this.model.get('voltage') < 22) {
      this.$el.html(this.templateDanger(this.model.toJSON()));
    } else {
      this.$el.html(this.templateNormal(this.model.toJSON()));
    }

    return this;
  }

});


module.exports = BatteryView;
