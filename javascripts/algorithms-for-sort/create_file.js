const fs = require('fs')
const ext = '.js'
let template = fs.readFileSync(`template${ext}`, { encoding: 'utf8' })

const [ , ,alg_name, arr_name] = process.argv

template = template.replace(/_1_/g, alg_name)
template = template.replace(/_2_/g, arr_name)

fs.writeFileSync(`${alg_name}${ext}`, template)
