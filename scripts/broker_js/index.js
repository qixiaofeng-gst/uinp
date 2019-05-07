// monitor changes on a file
//TODO sometime the cache may be corrupted: some file content is strangely empty

const fs = require('fs')
const path = require('path').posix
const util = require('util')

const read_into = (file_path, context) => {
  context[file_path] = {
    content: fs.readFileSync(file_path, { encoding: 'utf8' }),
    modify_time:  fs.statSync(file_path).mtimeMs,
  }
}
const parsers = [
  /\/\*PUT\s+(.+)\s+\*\//,
  /<!--PUT\s+(.+)\s+-->/,
]

const self = Symbol('self')
const merge = Symbol('merge')
const Assignable = () => {
  const mami = {}
  
  const absorb = val => {
    for (const k in val) {
      const v = val[k]
      if (undefined == val) {
        continue
      }
      mami[k] = v
    }
    return true
  }
  
  const clear = () => {
    for (const k in mami) {
      delete mami[k]
    }
  }
  
  return new Proxy(mami, {
    get: (_, key, __) => mami[key],
    set: (_, key, val, __) => {
      if (key === self) {
        clear()
        return absorb(val)
      }
      
      if (key === merge) {
        return absorb(val)
      }
      
      mami[key] = val
      return true
    },
  })
}
const rejoin_path = target => target.replace(/\\/g, path.sep)

const src_state = Assignable()

const start_watch = config => {
  const { input } = config
  
  if (false === fs.existsSync(input)) {
    console.error('Input file [', input, '] missing, could not start')
    return
  }
  
  const src_dir = path.dirname(input)
  make_output(config)
  
  console.log('The [ broker_js ] start watch')
  return fs.watch(src_dir, { recursive: true }, (e_type, file_name) => {
    /*
    e_type is either 'rename' or 'change'
    */
    const file_path = path.join(src_dir, rejoin_path(file_name))
    
    if (false === fs.existsSync(file_path)) {
      delete src_state[file_path]
      console.log('Remove:', file_path, new Date().toLocaleString())
      make_output(config)
      return
    }
    
    const stats = fs.statSync(file_path)
    const { mtimeMs } = stats
    
    src_state[file_path] = src_state[file_path] || {}
    if (src_state[file_path].modify_time == mtimeMs) {
      return
    }
    if (false === stats.isDirectory()) {
      read_into(file_path, src_state) 
    }
    
    console.log('Change:', e_type, file_path, new Date().toLocaleString())
    make_output(config)
  })
}

const combine = (in_file_path, context) => {
  const loop_detector = []
  const ref_count = {}
  
  console.log('Combine with file:', in_file_path)
  
  const inner_parse = (file_path, file_content) => {
    if (loop_detector.includes(file_path)) {
      console.warn('File [', file_path, '] is recursivly used, output might be corrupted. stack:', loop_detector)
      return file_content
    }
    
    const ref = ref_count[file_path] || ({ [self]: 0 })
    ref[self]++
    const { length: LL } = loop_detector
    if (LL > 0) {
      const last = loop_detector[LL - 1]
      const last_ref = ref[last] || 0
      ref[last] = last_ref + 1
    }
    ref_count[file_path] = ref
    
    loop_detector.push(file_path)
    const src_dir = path.dirname(file_path)
    let c = file_content
    for (const p of parsers) {
      const puts = c.match(new RegExp(p, 'gm'))
      if (null == puts) {
        continue
      }
      for (const put of puts) {
        const inner_file_path = path.resolve(path.sep, src_dir, put.match(p)[1]).substr(path.sep.length)
        if (false === fs.existsSync(inner_file_path)) {
          console.warn('File not exists:', inner_file_path)
          continue
        }
        if (false === context.hasOwnProperty(inner_file_path)) {
          read_into(inner_file_path, context)
        }
        c = c.replace(put, inner_parse(inner_file_path, context[inner_file_path].content))
      }
    }
    loop_detector.pop()
    return c
  }

  if (false === context.hasOwnProperty(in_file_path)) {
    read_into(in_file_path, context)
  }
  const result = inner_parse(in_file_path, context[in_file_path].content)

  for (const p in ref_count) {
    const ref = ref_count[p]
    const rc = ref[self]
    if (rc <= 1) {
      continue
    }
    console.warn('File [', p, '] is used', rc, 'times, used by:')
    for (const u in ref) {
      console.warn('    -', u, ref[u], 'times')
    }
  }
  
  return result
}

const make_output = config => {
  const { input, output } = config
  const out_file = path.join(output, path.basename(input))
  fs.writeFileSync(out_file, combine(input, src_state))
  console.log('Combined to', out_file)
}

module.exports = {
  start_watch
}
