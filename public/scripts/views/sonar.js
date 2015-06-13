'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var sonarTemplate = require('../templates/sonar.hbs');


var SonarView = Backbone.View.extend({

  el: $('#sonar'),

  template: sonarTemplate,

  initialize: function() {
    console.log('View sonar initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});

module.exports = SonarView;
