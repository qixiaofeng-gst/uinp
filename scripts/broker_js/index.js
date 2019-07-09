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
const parsers = {
  clike_put: {
    p: /\/\*PUT\s+(.+)\s+\*\//,
    cb_replacement: false,
  },
  html_put: {
    p: /<!--PUT\s+(.+)\s+-->/,
    cb_replacement: false,
  },
  require: {
    p: /require\('(.+)'\)/,
    cb_replacement: file_path => `modules['${(() => file_path)()}']`,
  },
}
const modules_placeholder = '/*MODULES*/'

const self = Symbol('self')
const rejoin_path = target => target.replace(/\\/g, path.sep)

//const inner_sm = 
const serialize_modules = modules => `/**
 * ATTENTION:
 * File is generated with broker_js, do not modify it manually.
 */
const modules = {}
${(() => {
  let result = ''
  for (const key in modules) {
    result += `modules['${key}']= (() => {\n${modules[key]}\n})()\n`
  }
  return result
})()}
//End modules\n\n`

const src_state = {}
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
    
    if (src_state[file_path] == mtimeMs) {
      return
    }
    src_state[file_path] = mtimeMs
    
    console.log('Change:', e_type, file_path, new Date().toLocaleString())
    make_output(config)
  })
}

const combine = (in_file_path, context, mds) => {
  const loop_detector = []
  const ref_count = {}
  
  console.log('Combine with file:', in_file_path)
  const de_modulize = content => {
    const exp = content.match(/module\.exports\s*=\s*/gm)
    if (null === exp) {
      return
    }
    if (1 < exp.length) {
      console.warn(`Current module has more than one 'module.exports =' statement, output might be invalid`)
      return
    }
    content = content.replace(exp[0], 'return (')
    content = content + ')'
    return content
  }
  
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
    for (const key in parsers) {
      const { p, cb_replacement } = parsers[key]
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
        if (cb_replacement) {
          if (false === mds.hasOwnProperty(inner_file_path)) {
            mds[inner_file_path] = de_modulize(inner_parse(inner_file_path, context[inner_file_path].content))
          }
          c = c.replace(put, cb_replacement(inner_file_path))
        } else {
          c = c.replace(put, inner_parse(inner_file_path, context[inner_file_path].content))
        }
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
  const modules = {}
  const { input, output } = config
  const out_file = path.join(output, path.basename(input))
  const combined = combine(input, {}, modules)
  if (combined.includes(modules_placeholder)) {
    fs.writeFileSync(out_file, combined.replace(modules_placeholder, serialize_modules(modules)))
  } else {
    fs.writeFileSync(out_file, serialize_modules(modules) + combined)
  }
  console.log('Combined to', out_file)
}

module.exports = {
  start_watch,
  make_output,
}
