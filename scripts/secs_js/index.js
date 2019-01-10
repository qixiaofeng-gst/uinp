const request = require('request')

const {
  fields
} = require('./libs/db')

const req = request.defaults({ jar: true })

const one_second = 1000
const one_minute = 60 * one_second
const one_hour = 60 * one_minute
const one_day = 24 * one_hour

const pankou = 'https://xueqiu.com/stock/pankou.json?symbol='
const quote = 'https://xueqiu.com/v4/stock/quotec.json?code='
const for_chart = 'https://xueqiu.com/stock/forchart/stocklist.json?symbol=x&period=1d'

const today_quotes = params => new Promise((resolve, reject) => {
  req({
    url: 'https://stock.xueqiu.com/v5/stock/chart/minute.json',
    json: true,
    qs: {
      symbol: 'SH600190',
      period: '1d'
    }
  }, (err, resp, body) => {
    console.log(body.data.items.length)
    console.log(body.data.items[0])
    console.log(new Date(body.data.items[0].timestamp).toLocaleString())
    resolve()
  })
})

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

let help_margin = 0
const set_fn_as_key = obj => {
  for (const k in obj) {
    obj[k].key = k
    help_margin = Math.max(help_margin, k.length)
  }
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
  fields: {
    name: '查看可获取的字段全集',
    desc: '列出所有字段中文名，代码名，字段数目统计，有变更的字段数目',
    handle: () => new Promise((resolve, reject) => {
      req({
        url: 'https://xueqiu.com/stock/screener/fields.json',
        qs: {
          category: 'SH',
          _: Date.now()
        },
        json: true
      }, (err, resp, body) => {
        if (err) {
          reject(err)
          return
        }
        const all_fields = []
        for (const category in body) {
          const arr = body[category]
          if (
            false === Array.isArray(arr) ||
            undefined === arr[0].name
          ) {
            console.log(`忽略复杂类别 ${category}`)
            continue
          }
          console.log(`${category} 有 ${arr.length} 个`)
          all_fields.splice(all_fields.length, 0, ...(arr.map(o => ({
            ...o,
            category
          }))))
        }
        console.log(`所有字段共计 ${all_fields.length} 个`)
        for (const { name, field } of all_fields) {
          console.log(`${name} ${field}`)
        }
        resolve()
      })
    })
  },
  list: {
    name: '列出目标',
    desc: '列出涨幅前列的股票代码以及相应涨幅',
    handle: params => new Promise((resolve, reject) => {
      let orderBy = 'percent'
      if (params && params.length) {
        orderBy = params[0]
      }
      req({
        url: 'https://xueqiu.com/stock/quote_order.json',
        json: true,
        qs: {
          stockType: 'sha', // 可用值 sha:沪A, sza:深A, cyb:创业板, zxb:中小板
          orderBy,
          order: 'desc', // 可用值 asc:升序, desc:降序
          size: 5,
          page: 1,
          column: 'symbol,name,current,chg,percent,volume,amount',
          _: Date.now()
        }
      }, (err, resp, body) => {
        if (err) {
          reject(err)
          return
        }
        console.log(body)
        resolve()
      })
    })
  },
  local: {
    name: '列出本地已存储数据',
    desc: '列出代码，以及已存储的时间范围',
    handle: params => new Promise((resolve, reject) => {
      resolve()
    })
  },
  fetch: {
    name: '取得历史数据',
    desc: '获取历史数据，输出条目总数、来源和耗时（第一次取得的将存到本地，以后再取就从本地取得），耗时操作',
    handle: params => new Promise((resolve, reject) => {
      req({
        url: 'https://stock.xueqiu.com/v5/stock/chart/kline.json',
        json: true,
        qs: {
          symbol: 'SZ300760',
          begin: 1547114028289,
          period: '1m',
          type: 'before',
          count: -142,
          indicator: 'kline,ma,macd,kdj,boll,rsi,wr,bias,cci,psy'
        }
      }, (err, resp, body) => {
        console.log(body.data.item.length)
        console.log(body.data.item[0])
        console.log(new Date(body.data.item[0].timestamp).toLocaleString())
        console.log(new Date(1547114028289).toLocaleString())
        resolve()
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

const readline = require('readline')
const read_cmd = () => {
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
  })
  rl.question('指令:', cmd => {
    rl.close()
    const parts = cmd.split(/\s+/)
    const cmd_str = parts.splice(0, 1)[0]
    const real_cmd = cmds[cmd_str]
    if (real_cmd) {
      const ts = Date.now()
      real_cmd.handle(parts).then(() => {
        console.log(`指令 ${cmd_str} 执行完毕，耗时 ${Date.now() - ts} 毫秒\n`)
      }).catch(err => {
        console.log(`指令 ${cmd_str} 执行失败，耗时 ${Date.now() - ts} 毫秒\n`)
      }).finally(() => {
        read_cmd()
      })
    } else {
      console.log(`不支持 ${cmd_str} 指令`)
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

req({
  url: 'https://xueqiu.com/hq/screener'
}, (err, resp, body) => {
  if (err) {
    console.log('初始化会话时发生错误:', err)
    return
  }
  console.log('证券 APP 命令行模式启动，help 指令可获取帮助\n')
  read_cmd()
})
