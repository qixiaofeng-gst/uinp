const
fs = require('fs'),
cstr = require('./colors'),

options = {
  epsilon: 1e-1,
},

epsilon = e => {
  options.epsilon = e
},

adjust_stack = e => {
  const
  rt = '\n',
  st = `${e.stack}`,
  ss = st.split(rt),
  cand = ss.splice(2, 3)
  
  let r = ss[0]
  for (const c of cand) {
    r += (rt + c)
  }
  r += (rt + '    ...')
  
  return r
},

/**
Assertion tool. Usage:
> the(a).eq(true) // Assert variable a equals true (behind use ===)
> the(b).eq(1.1)  // Assert the difference between number b and 1.1 is less than options.epsilon
*/
the = target => {
  const
  assert_eq = 'eq',
  check_float = val => {
    const type = 'number'
    
    if (typeof(val) == type && typeof(target) == type) {
      return Math.abs(t - v) < options.epsilon
    } else {
      return false
    }
  },
  eq_method = val => {
    if (check_float(val)) {
      return true
    } else {
      if (val === target) {
        return true
      } else {
        throw new Error(`Expect (${typeof(val)}) ${val}, get (${typeof(target)}) ${target}`)
      }
    }
  }
  
  return new Proxy({}, {
    get: (_, key, __) => {
      if (assert_eq == key) {
        return eq_method
      } else {
        throw new Error(`Test object does not have ${key} property.`)
      }
    },
    set: (_, key, val, __) => {
      throw new Error(`Test object does not support assignment statement.`)
    },
  })
},

/**
The tool for running test suite.
Could print out the statistics of result. (2/10 success, 8/10 failed, list out the name of failed test case)
*/
run = (suite, name) => {
  const type = 'function'
  
  let
  passed = [],
  failed = []
  
  for (const key in suite) {
    const case_cand = suite[key]
    if (typeof(case_cand) == type) {
      try {
        suite[key]()
        passed.push(cstr().green(`${name} - ${key}`).str())
      } catch (e) {
        failed.push(`Failed No.${failed.length + 1} [[${
          cstr().bright(name + ' - ' + key).str()
        }]] stacktrace:\n${
          cstr().red(adjust_stack(e)).str()
        }`)
      }
    }
  }
  
  const
  passed_quantity = passed.length,
  failed_quantity = failed.length,
  total = passed_quantity + failed_quantity
  
  if (0 === failed_quantity) {
    console.log(`Suite [${
      name
    }] ${
      cstr().green('all ' + total + ' cases passed').str()
    }.\n`)
  } else {
    console.log(`Suite [${
      cstr().bright(name).str()
    }], failed: ${
      cstr().red(failed_quantity).str()
    } / ${total}, passed: ${
      cstr().green(passed_quantity).str()
    } / ${total}`)
    console.log('Belows are details of failed:')
    for (const msg of failed) {
      console.log(msg)
    }
    console.log(`End summary of suite [${name}].\n`)
  }
},

/**
In the index.js under test directory, do:
run_dir(__dirname)
*/
run_dir = dir_path => {
  const
  ls = fs.readdirSync(dir_path),
  filter_index = 'index.js'

  for (const f of ls) {
    if (f == filter_index) {
      continue
    }
    run(require(`${dir_path}/${f}`), f)
  }
}

module.exports = {
  epsilon,
  the,
  run_dir,
}
