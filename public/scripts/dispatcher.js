'use strict';

/**
 * Event dispatcher to coordinate events among
 * different areas of the app.
 */

var Backbone = require('backbone');
var _ = require('underscore');


module.exports = _.clone(Backbone.Events);
