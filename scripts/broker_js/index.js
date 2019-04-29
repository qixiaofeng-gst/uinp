// monitor changes on a file
/*
  1. read config (input, output)
  2. parse input
  3. loop
  4. detect changes
  5. make output file
  
  XXX only one third done, about 2 hours left for this simple broker
*/

const fs = require('fs')
const path = require('path')
const config = require('./config')

const src_state = {}

const parse_input = () => {
  const { input } = config
  const src_dir = path.dirname(input)
  fs.watch(src_dir, { recursive: true }, (e_type, file_name) => {
    /*
    e_type is either 'rename' or 'change'
    */
    const file_path = path.join(src_dir, file_name)
    const stats = fs.statSync(file_path)
    
    if (stats.isDirectory()) {
      console.log(file_name, 'is directory')
      return
    }
    
    console.log(e_type, file_name, Date.now(), '[start]' + fs.readFileSync(file_path, { encoding: 'utf8' }) + '[end]')
  })
}

const load_files = () => {
  
}

const detect_changes = () => {
  
}

const make_output = () => {
  //console.log('just do it')
}

const start_detection_loop = cb => {
  setTimeout(() => {
    cb && cb()
    start_detection_loop(cb)
  }, 10)
}

parse_input()
load_files()
start_detection_loop(() => {
  detect_changes()
  make_output()
})
