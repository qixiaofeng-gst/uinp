const request = require('request')

const req = request.defaults({ jar: true })

const pankou = 'https://xueqiu.com/stock/pankou.json?symbol='
const quote = 'https://xueqiu.com/v4/stock/quotec.json?code='
const for_chart = 'https://xueqiu.com/stock/forchart/stocklist.json?symbol=x&period=1d'

const set_fn_as_key = obj => {
  for (const k in obj) {
    obj[k].key = k
  }
}
const cmds = {
  help: {
    name: '帮助命令',
    desc: '打印本帮助',
    handle: params => new Promise((resolve, reject) => {
      for (const k in cmds) {
        const c = cmds[k]
        console.log(`${k}, ${c.name}, ${c.desc}`)
      }
      resolve()
    })
  },
  list: {
    name: '列出目标',
    desc: '列出涨幅前列的股票代码以及相应涨幅',
    handle: params => new Promise((resolve, reject) => {
      resolve()
    })
  },
  fetch: {
    name: '取得历史数据',
    desc: '获取历史数据，输出条目总数、来源和耗时（第一次取得的将存到本地，以后再取就从本地取得），耗时操作',
    handle: params => new Promise((resolve, reject) => {
      resolve()
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
  rl.question('下面做:', cmd => {
    rl.close()
    const parts = cmd.split(/\s+/)
    const cmd_str = parts.splice(0, 1)[0]
    const real_cmd = cmds[cmd_str]
    if (real_cmd) {
      const ts = Date.now()
      real_cmd.handle(parts).then(() => {
        console.log(`做完了 ${cmd_str}，耗时 ${Date.now() - ts} 毫秒`)
        read_cmd()
      })
    } else {
      console.log(`不支持命令 ${cmd_str}`)
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
  console.log('证券 APP 命令行模式启动。help 指令可获取帮助。')
  read_cmd()
})
