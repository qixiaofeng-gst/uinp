#!/usr/bin/env node

const fs = require('fs')
const { execSync } = require('child_process')

const output_pack = execSync('npm pack')
console.log(output_pack.toString())

const contents = fs.readdirSync('./')
for (const c of contents) {
	if (c.endsWith('.tgz')) {
		const output_install = execSync(`npm install -g ${c}`)
		console.log(output_install.toString())
		fs.unlinkSync(c)
		console.log('Install done.')
		process.exit(0)
	}
}

console.log('Failed to install.')
