#!/usr/bin/env node

const cmd_read = (prompt, cb) => {
	const ctrl_c = 3
	const enter = 13
	const event_data = 'data'

	process.stdin.resume()
	process.stdin.setRawMode(true)
	process.stdout.write(prompt)
	let word = ''
	const on_data = data => {
		const input = data.toString()
		const code = input.charCodeAt()
		
		switch(code){
		case ctrl_c:
			console.log('\nCtrl+C pressed.')
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
	process.stdin.on(event_data, on_data)
}

const read_password = cb => cmd_read('Password plz:', pswd => {
	cmd_read('Repeat plz:', again => {
		if (pswd === again) {
			console.log('Correct.')
			cb && cb(pswd)
		} else {
			console.log('Not match.')
			read_password(cb)
		}
	})
})

read_password(code => {
	
})
