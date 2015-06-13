SERVER_SCRIPTS := $(shell find services -name '*.js' && find utils -name '*.js')
TESTS := $(shell find spec -name '*.js')
BINARIES := $(shell find bin)
PUBLIC_SCRIPTS := $(shell find public/scripts -name '*.js')

install:
	npm install

global:
	npm install -g browserify
	npm install -g watchify
	npm install -g jasmine
	npm install -g gulp

watch-js:
	@watchify -t browserify-handlebars ./public/scripts/app.js -o ./public/build/bundle.js -v

test:
	@jasmine

clean:
	rm -rf ./public/build

.PHONY: install
