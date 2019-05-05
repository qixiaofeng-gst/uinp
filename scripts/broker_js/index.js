// monitor changes on a file
/*
  1done. read config (input, output)
  2done. parse input
  3done. loop
  4done. detect changes
  5done. make output file
  
  only modulization left, about 1 hours work
*/

const fs = require('fs')
const path = require('path').posix
const util = require('util')

const read = file_path => fs.readFileSync(
  file_path,
  {
    encoding: 'utf8'
  }
)
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
      load_file(file_path, src_state) 
    }
    
    console.log('Change:', e_type, file_path, new Date().toLocaleString())
    make_output(config)
  })
}

const load_file = (in_file_path, context) => {
  const raw = read(in_file_path)
  context[in_file_path] = {
    content: raw,
    modify_time:  fs.statSync(in_file_path).mtimeMs,
  }
  
  console.log('Load file:', in_file_path)
  const parse_file = (fp, fc) => {
    const loop_detector = []
  
    const inner_parse = (file_path, file_content) => {
      if (loop_detector.includes(file_path)) {
        console.warn('File [', file_path, '] is recursivly used, output might be corrupted. stack:', loop_detector)
        return file_content
      }
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
            load_file(inner_file_path, context)
          }
          c = c.replace(put, inner_parse(inner_file_path, context[inner_file_path].content))
        }
      }
      loop_detector.pop()
      return c
    }
  
    return inner_parse(fp, fc)
  }
  
  return parse_file(in_file_path, raw)
}

const make_output = config => {
  const { input, output } = config
  const out_file = path.join(output, path.basename(input))
  fs.writeFileSync(out_file, load_file(input, src_state))
}

module.exports = {
  start_watch
}
