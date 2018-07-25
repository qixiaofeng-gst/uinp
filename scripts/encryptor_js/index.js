#!/usr/bin/env node

const fs = require('fs')

const cmd_read = (prompt, cb) => {
	const ctrl_c = 3
	const enter = 13
	const event_data = 'data'

	process.stdin.resume()
	if(!(process.stdin.setRawMode)) {
		process.stdout.write('Plz use this tool under CMD not under a cygwin.\n')
		process.exit()
	}
	process.stdin.setRawMode(true)
	let word = ''
	const on_data = data => {
		const input = data.toString()
		const code = input.charCodeAt()
		
		switch(code){
		case ctrl_c:
			process.stdout.write('\nCtrl+C pressed.')
			process.exit()
			break
		case enter:
			process.stdin.pause()
			process.stdin.setRawMode(false)
			process.stdin.removeListener(event_data, on_data)
			process.stdout.write('\n')
			cb && cb(word)
			break
		default:
			word += input
		}
	}
	process.stdout.write(prompt)
	process.stdin.on(event_data, on_data)
}

const read_password = cb => cmd_read('Password plz:', pswd => {
	cmd_read('Repeat plz:', again => {
		if (pswd === again) {
			process.stdout.write('Correct.\n')
			cb && cb(pswd)
		} else {
			process.stdout.write('Not match.\n')
			read_password(cb)
		}
	})
})

const { length } = process.argv
const file_ext = '.encrypted'
const target = process.argv[length - 1]
if (fs.existsSync(target) && false === target.endsWith('index.js')) {
	read_password(code => {
		const mask = Buffer.from(code)
		console.log('==== code:', mask[0], mask.length)
		const rs = fs.createReadStream(target)
		const ws = fs.createWriteStream(target + file_ext)
		const trans = () => {
			const data = rs.read(1)
			if (data) {
				data[0] = (data[0] ^ mask[0])
				ws.write(data, trans)
			} else {
				rs.destroy()
				ws.destroy()
			}
		}
		rs.on('readable', trans)
	})
} else {
	process.stdout.write('The last argument has to be a existing file.')
}
