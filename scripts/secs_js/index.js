const request = require('request')

const req = request.defaults({ jar: true })

req({
  url: 'https://xueqiu.com/hq/screener'
}, (err, resp, body) => {
  if (err) {
    console.log('初始化会话时发生错误:', err)
    return
  }
  req({
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
})
