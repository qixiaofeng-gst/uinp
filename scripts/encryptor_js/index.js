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

const read_password = (cb, repeat=true) => cmd_read('Code plz:', pswd => {
	if (repeat) {
		cmd_read('Repeat plz:', again => {
			if (pswd === again) {
				process.stdout.write('Correct.\n')
				cb && cb(pswd)
			} else {
				process.stdout.write('Not match.\n')
				read_password(cb)
			}
		})
	} else {
		cb && cb(pswd)
	}
})

const { length } = process.argv
const file_ext = '.encrypted'
const target = process.argv[length - 1]
const is_encrypted = target.endsWith(file_ext)
const tmp = {}
const output = is_encrypted ? (
	tmp.output = target.substr(0, target.length - file_ext.length),
	(fs.existsSync(tmp.output) ? (tmp.output + '.decrypted') : tmp.output)
) : (target + file_ext)
if (fs.existsSync(target) && false === target.endsWith('index.js')) {
	read_password(code => {
		const mask = Buffer.from(code)
		const { length: len } = mask
		const rs = fs.createReadStream(target)
		const ws = fs.createWriteStream(output)
		const trans = () => {
			const data = rs.read(len)
			if (data) {
				for (let i = data.length; i > 0; --i) {
					const idx = i - 1
					data[idx] = (data[idx] ^ mask[idx])
				}
				ws.write(data, trans)
			} else {
				rs.destroy()
				ws.destroy()
			}
		}
		rs.on('readable', trans)
	}, false === is_encrypted)
} else {
	process.stdout.write('The last argument has to be a existing file.')
}
