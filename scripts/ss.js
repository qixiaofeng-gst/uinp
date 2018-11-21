/** Session Class Definition. */

const { md5 } = require('./util')

const _one_minute = 60000
/**
 * @param expire_time Time in minutes.
 */
const Session = function(expire_time){
	const missing_key_msg = 'A valid session key is required.'
	const exp_time = (expire_time * _one_minute)
	const ss_container = {}
	proto_ro_(this, 'expire_time', () => exp_time)
	proto_m_(this, 'check', function(ss_key) {
		return ss_container.hasOwnProperty(ss_key)
	})
	proto_m_(this, 'get', function(ss_key) {
		if (!ss_key) {
			throw new Error(missing_key_msg)
		}
		return ss_container[ss_key]
	})
	proto_m_(this, 'set', function(ss_key, ss_obj) {
		if (!ss_key) {
			throw new Error(missing_key_msg)
		}
		if (!ss_obj) {
			throw new Error('A valid session object is required.')
		}
		ss_obj.start_time = Date.now()
		const key = md5(`${ss_key}${ss_obj.start_time}`)
		ss_container[key] = ss_obj
		return key
	})
	proto_m_(this, 'clear', function(ss_key) {
		// Clear the specified.
		if (ss_key) {
			delete ss_container[ss_key]
		}
		// Clear expired.
		const expired = []
		for (const key in ss_container) {
			const { start_time } = ss_container[key]
			if (exp_time < (Date.now() - start_time)) {
				expired.push(key)
			}
		}
		for (const r of expired) {
			delete ss_container[r]
		}
		return this
	})
	proto_m_(this, 'renew', function(ss_key) {
		const ss_obj = ss_container[ss_key]
		if (!ss_key || !ss_obj) {
			throw new Error(missing_key_msg)
		}
		ss_obj.start_time = Date.now()
		return ss_obj
	})
}

const proto_ro_ = (obj, key, get) => ( // Add readonly field for prototype.
	Object.defineProperty(
		obj, key,
		{
			get, set() { throw new Error(`Cannot assign to ${key}.`) }
		}
	)
)

const proto_m_ = (obj, key, m) => { // Add method for prototype.
	const is_function = (m instanceof Function)
	if (!is_function) {
		throw new Error(`Must give a function to ${key}.`)
	}
	
	return Object.defineProperty(
		obj, key,
		{ value: m }
	)
}

module.exports = Session
