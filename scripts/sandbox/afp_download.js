const fs = require('fs')

const prefix = 'http://videocdn.licaiedu.com/licaiedu3/AFP2019/'
const suffix = '.mp4'
const desc = {
  AA: 9, //金融理财概述与CFP认证制度
  AB: 9, //经济学基础知识
  AC: 13,//货币时间价值与理财资讯平台的运用
  BE: 13,//金融理财法律
  AE: 15,//家庭财务报表编制与财务诊断
  AF: 10,//居住规划
  BD: 4, //子女教育金规划
  AG: 11,//投资基础
  AH: 2, //现金及其等价物
  AI: 8, //债券市场与债券投资
  AJ: 8, //股票市场与股票投资
  AK: 3, //期权基础知识
  AL: 3, //外汇与汇率
  AM: 3, //贵金属投资基础
  AN: 9, //基金投资
  AO: 9, //理财产品投资
  AP: 10,//投资组合理论
  AQ: 3, //投资人特征分析
  AR: 7, //资产配置与绩效评估
  AS: 3, //风险与风险管理
  AT: 8, //保险基本原理
  AU: 10,//人寿保险
  AV: 5, //年金保险
  AW: 17,//员工福利
  AX: 8, //退休规划
  AY: 14,//个人所得税制度
  AZ: 11,//个人所得税优化
  BA: 6, //信用与债务管理
  BB: 11,//综合理财规划原理
  BC: 13,//理财规划软件案例示范
}

let result = ''
for (const key in desc) {
  const count = desc[key]
  for (let i = 1; i <= count; ++i) {
    result += `${prefix}${key}${i}${suffix}\n`
  }
  result += '\n'
}
fs.writeFileSync('afp_urls.txt', result)
