#!-*- coding: utf-8 -*-

import urllib2
import urllib
from bs4 import BeautifulSoup
import re
import time
import sys

reload(sys)
sys.setdefaultencoding( "utf-8" )

class DownloadSong(object):
    def __init__(self,base_url):
        self.url = base_url
        self.music_url = 'http://mp3-cdn2.luoo.net/low/luoo/'
        self.pageIndex = 1
        self.headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.115 Safari/537.36'
        }

    #获取页面的源码
    def getPage(self, index, vol_url):
        if index != 0:
            url = self.url + '?p=' + str(index)
        else:
            url = vol_url
        # try:
        request = urllib2.Request(url, headers=self.headers)
        response = urllib2.urlopen(request).read()
        soup = BeautifulSoup(response, 'html.parser')
        return soup
        # except urllib2.URLError,e:
        #     if hasattr(e, 'reason'):
        #         print(u'链接失败,失败原因', e.reason)
        #         return None

    #获取总页面数
    def getTotalPage(self):
        soup = self.getPage(self.pageIndex, '')
        if not soup:
            return None
        totalPage = soup.find_all('a', class_='page')[-1].get_text().strip()
        return totalPage

    #处理歌曲的名称
    def delSongName(self, songName):
        return songName.split('.')[1].lstrip().encode('utf-8')

    #获取每个期刊URL与频道的对应关系
    def getReation(self, pageIndex):
        soup = self.getPage(pageIndex, '')
        vols = {}
        pattern = re.compile('[0-9]+')
        if not soup:
            return None
        vol_lists = soup.find_all('div', class_='meta')
        for vol in vol_lists:
            vol1 = re.search(pattern, str(vol.a.get_text())).group()
            vols[vol1.strip()] = vol.a['href']
        return vols

    #获取每首歌的名称和url
    def getSongInfo(self, vols):
        songInfos = {}
        for vol in vols.keys():
            url = vols[vol]
            soup = self.getPage(0, url)
            total = len(soup.find_all('li', class_='track-item'))
            songNames = soup.find_all('a', class_='trackname')
            for i in range(1, total+1):
                songName = self.delSongName(songNames[i - 1].get_text())
                if i < 10:
                    songURL = self.music_url + 'radio' + str(vol).lstrip('0') + '/0' + str(i) + '.mp3'
                else:
                    songURL = self.music_url + 'radio' + str(vol).lstrip('0') + '/' + str(i) + '.mp3'
                songInfos[songName] = songURL
        return songInfos

    #下载歌曲
    def downloadSong(self):
        totalPage = self.getTotalPage()
        for pageIndex in range(1, int(totalPage)+1):
            vols = self.getReation(pageIndex)
            songInfos = self.getSongInfo(vols)
            for songName, songURL in songInfos.items():
                time.sleep(5)#适当的减慢下载速度，不要给人家服务器造成压力。
                print('%s 正在下载中。。。' %(songName))
                try:
                    data = urllib2.urlopen(songURL).read()
                except urllib2.URLError:
                    print("######链接不存在，继续下载下一首########")
                with open (('D:\\test\\song\\%s.mp3' %(songName)).decode('utf-8'), 'wb') as f:
                    f.write(data)

if __name__ == '__main__':
    url = 'http://www.luoo.net/music/classical'   
    print("downloadsong:",downloadsong)
    downloadsong.downloadSong() 