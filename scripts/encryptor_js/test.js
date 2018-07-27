#!/usr/bin/env node

console.log('The process.stdin stream not work as expected on cygwin.')

const fs = require('fs')
const { execSync: es } = require('child_process')

const files = fs.readdirSync('.')
const space = ' '
for (const f of files) {
	const idx = f.indexOf(space)
	if (idx >= 0) {
		const updated = f.substr(0, idx) + f.substr(idx + 1, 10)
		console.log(f, '>', updated)
		es(`mv '${f}' ${updated}`)
	}
}
