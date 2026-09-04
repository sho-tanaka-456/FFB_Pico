
Constant Forceのみ実装
Windowsにおいては概ね動作することを確認

既知の問題
・CDCを有効化すると帯域が使われるのか、magunitudeなどの受信周期が明らかに長くなる（ガリガリする感じになる、0点がモーター１回転分ずれる。）
・fftestにおいてmagnitude取得できず
・RaceRoomではffb_activeにもならない
・F1 2016でも同様にffb_activeにならずカクつく（おそらく返答失敗）

