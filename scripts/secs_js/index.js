const request = require('request')
const moment = require('moment-timezone')

// M(B/S)C: Market buy/sell commission
// L(B/S)C: Limit buy/sell commission
// EX: Exchange
// SS: Session, a buy-then-sell turn
// p_pre: price prediciton: fut_t, p_rng and exp_p
// fut_t: future timestamp
// p_rng: price range
// exp_p: price expectation
// c_m_p: current market price
// e_p_r: expected profit ratio, = e_c_r + n_p_r
// e_c_r: exchange cost ratio
// n_p_r: net profit ratio
// stp_p: stop price
// exc_p: exchange price
// pp_sq: p_pre sequence
/*
Logic for buy:
1. Contineously check c_m_p and update the p_pre
   - If (exp_p - c_m_p) > e_p_r, then set LBC
   - If there is a valley on the pp_sq, then set LBC //TODO here need more details
Logic for sell:
1. Immediately set LSC with (exc_p * (1 + e_p_r)) after EX
2. If LSC is conducted, then SS complete, else contineously check c_m_p and update the p_pre
   - If (exp_p < stp_p) && (c_m_p < stp_p), then set a LC with max(c_m_p, exp_p)
*/

const tz = {
  cn: 'Asia/Shanghai',
  us: 'America/New_York'
}
const df = 'YYYYMMDD' // date format
const get_ts = date => {
  return parseInt(date)
}
const last_30_days = () => {
  const current = moment().tz(tz.us)
  const open_days = []
  for (let i = 1; i < 30; ++i) {
    current.subtract(1, 'd')
    if (0 === current.day() || 6 === current.day()) {
      continue
    }
    open_days.push(current.format(df))
  }
  return open_days
}
const fetch_history = (symbol, date) => new Promise((resolve, reject) => {
  date = parseInt(date)
  histories.get({
    date,
    symbol
  }).then(history => {
    if (history) {
      console.log(`自缓存中取得 ${symbol} ${date}`)
      resolve(history.data)
    } else {
      get(`/stock/${symbol}/chart/date/${date}`).then(data => {
        if (data.length > 0) {
          console.log(`取得 ${symbol} ${date} 成功`)
          histories.insert({
            date,
            symbol,
            data
          }).then(result => {
            console.log(`缓存 ${symbol} ${date} 成功`)
            resolve(data)
          }).catch(err => {
            console.log(`缓存 ${symbol} ${date} 失败`)
            resolve(data)
          })
        } else {
          const msg = `取得 ${symbol} ${date} 数据为空`
          console.log(msg)
          reject({ msg })
        }
      }).catch(err => {
        console.log(`取得 ${symbol} ${date} 失败`)
        console.log(err)
        reject(err)
      })
    }
  }).catch(err => {
    reject(err)
  })
})
const fetch_last_30 = symbol => new Promise(resolve => {
  const results = []
  const l30d = last_30_days()
  const recursively_do = index => {
    index = index || 0
    if (index < l30d.length) {
      fetch_history(symbol, l30d[index]).then(data => {
        results.push(data)
        recursively_do(index + 1)
      }).catch(err => {
        recursively_do(index + 1)
      })
    } else {
      resolve(results)
      return
    }
  }
  recursively_do()
})

const {
  set_fn_as_key,
  calc_margin,
  calc_margin_arr
} = require('./libs/obj_utils')
const {
  symbol_types
} = require('./libs/consts')
const {
  histories
} = require('./libs/db')

const req = request.defaults({ jar: true })
const get = (api, params) => new Promise((resolve, reject) => {
  req({
    url: `https://api.iextrading.com/1.0${api}`,
    json: true,
    qs: {
      ...params
    }
  }, (err, resp, body) => {
    if (err) {
      reject(err)
    } else {
      resolve(body)
    }
  })
})

const one_second = 1000
const one_minute = 60 * one_second
const one_hour = 60 * one_minute
const one_day = 24 * one_hour

const spaces = num => {
  let s = ''
  for (let i = 0; i < num; ++i) {
    s += ' '
  }
  return s
}
const fill_to = (str, num) => {
  return str + spaces(num - str.length)
}

const cmds = {
  help: {
    name: '帮助命令',
    desc: '打印本帮助',
    handle: params => new Promise((resolve, reject) => {
      for (const k in cmds) {
        const c = cmds[k]
        console.log(`${fill_to(k, help_margin)} ${c.name}`)
        console.log(`${spaces(help_margin)} - ${c.desc}`)
      }
      resolve()
    })
  },
  types: {
    name: 'List Types Command',
    desc: 'List all symbol types',
    handle: () => new Promise(resolve => {
      for (const k in symbol_types) {
        const t = symbol_types[k]
        console.log(`${fill_to(k, type_margin)} ${t.desc}`)
        if (t.explain) {
          console.log(`${spaces(type_margin)} - ${t.explain}`)
        }
      }
      resolve()
    })
  },
  symbols: {
    name: 'List symbols',
    desc: 'Retreive the symbol represented in INET(Nasdaq Integrated symbology), print the count, cost 3~15 seconds',
    handle: params => new Promise((resolve, reject) => {
      get('/ref-data/symbols').then(data => {
        console.log('Count:', data.length)
        resolve()
      }).catch(err => {
        reject(err)
      })
    })
  },
  list: {
    name: 'List Command',
    desc: 'List top 10 symbols according to the passed parameter',
    handle: params => new Promise((resolve, reject) => {
      let sorter = 'gainers' // available values: mostactive, gainers, losers, iexvolume, iexpercent, infocus
      if (params && params.length) {
        sorter = params[0]
      }
      get(`/stock/market/list/${sorter}`).then(data => {
        const margin = calc_margin_arr(data, 'symbol')
        for (let i = 0; i < data.length; ++i) {
          const q = data[i]
          console.log(`${fill_to(q.symbol, margin)} ${q.changePercent} ${q.change}`)
        }
        resolve()
      }).catch(err => {
        reject(err)
      })
    })
  },
  fetch: {
    name: 'Fetch History Command',
    desc: 'Fetch history of specific symbol at specific date.',
    handle: params => new Promise((resolve, reject) => {
      let symbol = 'aapl' // yeah it is the iphone's apple
      if (params && params.length) {
        symbol = params[0]
      }
      fetch_last_30(symbol).then(results => {
        console.log(`共取得 ${results.length} 天数据`)
        resolve(results)
      })
    })
  },
  local: {
    name: '列出本地已存储数据',
    desc: '列出代码，以及已存储的时间范围',
    handle: params => new Promise((resolve, reject) => {
      histories.exec().then(({ table, log_ts }) => table.aggregate([
        { $group: { _id: '$symbol', day_count: { $sum: 1 } } }
      ]).toArray().then(results => {
        for (const { _id, day_count } of results) {
          console.log(`本地 ${_id} 有 ${day_count} 天`)
        }
        resolve(results)
      }).catch(err => {
        reject(err)
      })).catch(err => {
        reject(err)
      })
    })
  },
  bt: {
    name: '回测',
    desc: '使用历史数据测试策略，传一个代码作为参数，属于耗时操作',
    handle: params => new Promise((resolve, reject) => {
      resolve()
    })
  },
  exit: {
    name: '退出',
    desc: '退出进程',
    handle: params => new Promise((resolve, reject) => {
      process.exit(0)
      resolve()
    })
  },
}
set_fn_as_key(cmds)
const help_margin = calc_margin(cmds)
const type_margin = calc_margin(symbol_types)

const readline = require('readline')
const read_cmd = () => {
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
  })
  rl.question('Command:', cmd => {
    rl.close()
    const parts = cmd.split(/\s+/)
    const cmd_str = parts.splice(0, 1)[0]
    const real_cmd = cmds[cmd_str]
    if (real_cmd) {
      const ts = Date.now()
      real_cmd.handle(parts).then(() => {
        console.log(`Command ${cmd_str} 执行完毕，耗时 ${Date.now() - ts} 毫秒\n`)
      }).catch(err => {
        console.log(`Command ${cmd_str} 执行失败，耗时 ${Date.now() - ts} 毫秒\n`)
        console.log(err)
      }).finally(() => {
        read_cmd()
      })
    } else {
      console.log(`不支持 ${cmd_str} Command`)
      read_cmd()
    }
  })
}

const test_method = () => req({
  url: 'https://xueqiu.com/stock/screener/screen.json',
  qs: {
    category: 'SH',
    orderby: 'chgpct1m',
    order: 'desc',
    current: 'ALL',
    pct: 'ALL',
    page: 1,
    chgpct1m: '0_142.9878',
    _: Date.now()
  },
  json: true
}, (err, resp, body) => {
  console.log(body.list.length, '====', body.list.map(o => o.chgpct1m))
  const { symbol } = body.list[0]
  req({
    url: 'https://stock.xueqiu.com/v5/stock/chart/minute.json',
    qs: {
      symbol,
      period: '5d'
    },
    json: true
  }, (err, resp, body) => {
    console.log(body.data.items.length, '==== inner')
  })
})

console.log('证券 APP 命令行模式启动，help Command可获取帮助\n')
read_cmd()
