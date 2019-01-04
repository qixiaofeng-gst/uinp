# 龙虎榜调仓信息有效性分析
# 分析对象： https://xueqiu.com/5171159182/96724512

####### ####### ####### ####### ####### ####### #######
# Plot industry.
pubDate = 'publishDate'
brokerSecIDs = '002797.XSHE,000686.XSHE,600958.XSHG,601555.XSHG,601901.XSHG,\
601788.XSHG,000776.XSHE,000750.XSHE,600109.XSHG,601211.XSHG,002736.XSHE,\
000728.XSHE,600837.XSHG,600909.XSHG,601688.XSHG,002500.XSHE,000166.XSHE,\
601099.XSHG,002673.XSHE,600369.XSHG,601377.XSHG,000783.XSHE,600999.XSHG,\
601878.XSHG,601881.XSHG,600030.XSHG,601375.XSHG'

brokers = DataAPI.FdmtISGet(secID=brokerSecIDs, reportType='Q1,Q2,Q3,Q4')
filtered = brokers[brokers[pubDate].str[0:4] == brokers['endDate'].str[0:4]]
grouped = filtered.groupby('secID')

for name, group in grouped:
    print group['secShortName'].iloc[0]
    toBeShown = group.sort_values(pubDate)
    toBeShown.plot.bar(x=pubDate, y='tRevenue', figsize=(14, 2), title=name)
    toBeShown.plot.bar(x=pubDate, y='NIncome', figsize=(14, 2), title=name)

####### ####### ####### ####### ####### ####### #######
# Plot single equity.
def plot(pdData, name):
    pdData.plot(x='tradeDate', y=name, figsize=(20, 2), grid=True)

def plotFor(secID):
    equData = DataAPI.MktEqudGet(secID=secID)
    plot(equData, 'closePrice')
    plot(equData, 'PB')
    plot(equData, 'PE')
    plot(equData, 'marketValue')
    plot(equData, 'turnoverRate')

plotFor('601857.XSHG')

####### ####### ####### ####### ####### ####### #######
# Plot market.
def plotTurnoverRateRank(tDate):
    toRate = 'turnoverRate'
    sID = 'secID'
    today = DataAPI.MktEqudGet(tradeDate=tDate)
    today = today[today[toRate] > 0]
    today = today.sort_values(toRate)
    today.head(180).plot.barh(x=sID, y=toRate, figsize=(6, 30), grid=True)
    today.tail(180).plot.barh(x=sID, y=toRate, figsize=(3, 30), grid=True)

plotTurnoverRateRank('20171206')

''' Colormap possible values are: Accent, Accent_r, Blues, Blues_r, BrBG, BrBG_r, BuGn, BuGn_r, BuPu, BuPu_r, CMRmap, CMRmap_r, Dark2, Dark2_r, GnBu, GnBu_r, Greens, Greens_r, Greys, Greys_r, OrRd, OrRd_r, Oranges, Oranges_r, PRGn, PRGn_r, Paired, Paired_r, Pastel1, Pastel1_r, Pastel2, Pastel2_r, PiYG, PiYG_r, PuBu, PuBuGn, PuBuGn_r, PuBu_r, PuOr, PuOr_r, PuRd, PuRd_r, Purples, Purples_r, RdBu, RdBu_r, RdGy, RdGy_r, RdPu, RdPu_r, RdYlBu, RdYlBu_r, RdYlGn, RdYlGn_r, Reds, Reds_r, Set1, Set1_r, Set2, Set2_r, Set3, Set3_r, Spectral, Spectral_r, Wistia, Wistia_r, YlGn, YlGnBu, YlGnBu_r, YlGn_r, YlOrBr, YlOrBr_r, YlOrRd, YlOrRd_r, afmhot, afmhot_r, autumn, autumn_r, binary, binary_r, bone, bone_r, brg, brg_r, bwr, bwr_r, cool, cool_r, coolwarm, coolwarm_r, copper, copper_r, cubehelix, cubehelix_r, flag, flag_r, gist_earth, gist_earth_r, gist_gray, gist_gray_r, gist_heat, gist_heat_r, gist_ncar, gist_ncar_r, gist_rainbow, gist_rainbow_r, gist_stern, gist_stern_r, gist_yarg, gist_yarg_r, gnuplot, gnuplot2, gnuplot2_r, gnuplot_r, gray, gray_r, hot, hot_r, hsv, hsv_r, inferno, inferno_r, jet, jet_r, magma, magma_r, nipy_spectral, nipy_spectral_r, ocean, ocean_r, pink, pink_r, plasma, plasma_r, prism, prism_r, rainbow, rainbow_r, seismic, seismic_r, spectral, spectral_r, spring, spring_r, summer, summer_r, terrain, terrain_r, viridis, viridis_r, winter, winter_r '''
