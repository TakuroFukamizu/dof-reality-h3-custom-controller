# DIN の 5V が AXP2101 の OFF 状態を固定する

Issue #1 の症状は二つの問いに分けられる。一つ目の「なぜ落ちたまま戻らないのか」は、回路図とデータシートでほぼ説明がつく。CoreS3 SE の M-Bus 5V（コア側ネット `BUS_OUT`）と USB-C の VBUS は、どちらも ON 状態の負荷スイッチ ME1502 を通って AXP2101 の VBUS ピンに直結している。いまのファーム設定（`output_power=false` と `setUsbOutput(false)`）は、AW9523 のリセット既定値と同じ「両方 ON」の状態をそのまま残す。このため AXP2101 が何かの理由で OFF 状態に入ると、DIN Base の 5V が VBUS を保ち続け、自動再起動の条件である「VBUS の Low→High」が来ない。USB を抜くだけでは戻らず、両方を抜いて約 20 s 待つと POR がかかって復帰する。観察はこの説明と一致する。赤 LED が点いたままだったので、DIN Base の降圧は生きていた。止まったのはコア側である。二つ目の「約 85 s 後に何が AXP2101 を OFF にしたのか」は、どの一次資料でも説明できず、過去事例にも同じ遅れ方は見つからなかった。最有力は、DIN の 5V と Mac の VBUS が約 140 mΩ で並列になって起きる二電源の競合である。ほかに、バッテリーを外した VBAT ノードを DIN Base の充電 IC が駆動し続ける問題、フラッシュ済みのファームが古く DCDC 低電圧ラッチが有効なままである可能性が残る。原因は AXP2101 の REG 0x21（電源断要因）が名指ししてくれるが、このレジスタは POR でしか消えない。したがって次に落ちたときは、**ケーブルを抜く前に電源キーを押し、その起動で REG 0x20/0x21 を読む**。これが最も安く、最も判別力のある一手である。その準備として、記録用のファームを先に書き込んでおく必要がある。

本文では、一次資料（回路図、データシート、ライブラリのソース）で確かめた点を【検証】、それらから推論した点を【推定】と書き分ける。記号のない文は、両者をつないだ説明である。

## 両電源は約 140 mΩ で並列につながり、ファームでは切り離せない

CoreS3 SE の回路図 6 ページには負荷スイッチが 4 個ある。M-Bus 5V の `BUS_OUT` は U19（ME1502**C**、EN 負論理、EN=`BUS_OUT_EN`）を通って AXP2101 の `VBUS` へ、USB-C の `VUSB` は U18（ME1502C、EN=`USB_OTG_EN`）を通って同じ `VBUS` へ入る。ダイオード OR も調停回路もない【検証】([SE 回路図 p6](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_06.png))。無印 CoreS3 の同ページも、部品、ネット、抵抗値まで同一である【検証】([CoreS3 回路図](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf))。二つの EN は AW9523B（0x58）の P0_1 と P0_5 から出ている【検証】([SE 回路図 p5](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_05.png))。AW9523B はパワーオン時に全出力が 0 になり【検証】([AW9523B datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/core/CoreS3/AW9523B-EN.pdf))、M5Stack 公式の電源方向図は `BUS_OUT_EN=0, USB_OTG_EN=0` を「USB INPUT / BUS INPUT」として図示している【検証】([電源方向図](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/core/CoreS3/%E7%94%B5%E6%BA%90.png))。両入力の同時使用に関する注意書きは、CoreS3 SE と DIN Base のどちらの docs にもない【検証】([CoreS3 SE docs](https://docs.m5stack.com/en/core/M5CoreS3%20SE)、[DIN Base docs](https://docs.m5stack.com/en/base/DIN%20BASE))。

本プロジェクトのファームは `cfg.output_power = false` で `M5.begin()` を呼び、続けて `M5.Power.setUsbOutput(false)` を呼ぶ（`moter-controler/src/main.cpp` L773-791）。M5Unified 0.2.26 では、前者が `BUS_OUT_EN=0, BOOST_EN=0`、後者が `USB_OTG_EN=0` を書く【検証】([Power_Class.inl L967-1059](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L967-L1059)、[L1414-1433](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1414-L1433))。結果は **U18 と U19 が同時に ON** で、AW9523 の既定値と変わらない。ファームが並列状態を選んだのではなく、変えなかっただけである。ME1502 の ON 抵抗は 70 mΩ で、逆電流を防ぐのは「芯片关断时」（**オフ時**）だけとデータシートに書かれている。絶対最大定格は VOUT ≤ VIN である【検証】([ME1502 datasheet](https://static.chipdip.ru/lib/087/DOC043087044.pdf))。したがって DIN の 5V と Mac の VBUS は約 140 mΩ のスイッチ越しに直結し、電圧の高い側が、相手側のスイッチを逆方向に通って低い側へ電流を押し込める【推定】。ON 時の逆電流に限流や過温保護が効くかどうかは、データシートに記載がない。M5Unified のメンテナ自身も、ソースのコメントで「BUS_OUT_EN=0 の状態では USB VBUS が BUS_OUT へ流れ込み続ける」と、この経路の存在を認めている【検証】([Power_Class.inl L988-991](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L988-L991))。

DIN Base 側にも逆流防止はない。DC ジャックの 9-24 V は DSS34 とスライドスイッチを通り、常時 ON の同期整流降圧 SY8303AIC に入る。その出力 DIN `BUS_5V` は、M-Bus の 5V ピンと Grove の 5V に直結している【検証】([DIN Base 回路図 V1.1](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf))。帰還抵抗 R1 120 kΩ / R3 16 kΩ と VOUT = 0.6×(1+R1/R3) から、設定値は約 **5.10 V** になる【推定、計算】([SY8303A datasheet](https://static.chipdip.ru/lib/484/DOC043484845.pdf))。名前の罠に注意したい。DIN 側の `BUS_5V` はコア側の `BUS_OUT` と同じネットである。コア側の `BUS_5V` は内蔵昇圧 SY7088 の出力で、別物である。

ソフトウェアの設定では、この並列は解けない。`BUS_OUT_EN=1` にすれば U19 は OFF になるが、同時に U14 が ON になり、内蔵昇圧が `BUS_OUT` 上で DIN の 5V とぶつかる。`USB_OTG_EN=1` にすれば U18 は OFF になるが、U17 が ON になり、Mac の VBUS へ給電する向きになる【推定、p6 の結線から】。そもそも `output_power=true` は、バッテリーがなく、TS（= `BUS_OUT`/2）が 2.0 V を超えている間、M5Unified が取り消す【検証】([Power_Class.inl L1062-1107](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1062-L1107))。M5Stack も「どのコネクタが実際の電源かをファームが確実に判定する手段をハードが持たない」と述べている【検証】([M5CoreS3 Issue #72](https://github.com/m5stack/M5CoreS3/issues/72))。USB のみで 15 分以上問題がなかったのも、12V のみで起動できたのも、電源が 1 系統なので競合が起きないからだと説明できる。12V のみのときは U18 の VIN が 0 V なので、UVLO で OFF になり逆流も止まる【推定】。

AXP2101 の振る舞いも、「戻らない」理由と合っている。データシート §6.5.4.3 が挙げる電源断の要因は、キー長押し、ソフト電源断、VSYS < VOFF または VBUS > 7 V、DCDC 15 % 低下、DCDC 過電圧、ダイ 145 ℃ の 6 項目で、REG 0x21 はこれに EN Low を加えた 8 ビットで要因を区別する。OFF 状態からの起動要因は、キー（ONLEVEL）と「VBUS low go high」などで、要因は REG 0x21 と REG 0x20 に残る。**REG 0x21 の Reset 列は「POR」**である【検証】([AXP2101 datasheet V1.0](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.5.4.2-3、§6.13.2.17-18)。これらをつなぐと、AXP2101 が OFF になった後は次のように進む【推定】。DCDC1 が止まって 3.3V が落ち、ESP32、LCD、AW9523 が停止して USB デバイスが消える。それでも U19 越しの DIN 5V が VBUS を UVLO（3.75 V）より上に保つので、PMIC は通電したまま OFF 状態にとどまる。USB を抜いても VBUS は落ちず、起動のきっかけになるエッジが来ない。両方を抜くと、220 µF 級のバルク容量が放電して POR がかかる。約 20 s という時間は、この放電時間として矛盾しない（未測定）。ESP32-S3 のブラウンアウト検出はチップを自動で再起動させる仕組みなので【検証】([ESP-IDF Kconfig](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/kconfig-reference.html))、画面が暗いまま戻らないという症状とは合わない。

DIN Base の赤 LED が点いたままだったことも重要な証拠である。D2（1615RG 二色）の赤は TP4057 の CHRG 表示で、電源は Q1 SI2301 で `BUS_5V` を切り替えた `VCC_5V` から取っている【検証】([DIN Base 回路図](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf))。つまり、コアが落ちた時点でも DIN の 5V は出ていた。降圧の保護動作は、故障の主因から外せる【推定】。

## 85 秒の引き金はどの資料でも説明できない。仮説を 9 つに順位づける

引き金の候補を、Issue の観察と照らして並べたのが下の表である。電源断の要因のうち、DCDC 低電圧（REG21 b5）は、**現在の libdeps（M5Unified 0.2.26/0.2.28）でビルドしたファームなら** `bitOff(0x23, 0x1F)` で無効になっている【検証】([Power_Class.inl L415](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L415))。AXP2101 のウォッチドッグは既定で無効（REG18[0]=0）で、M5Unified もそれを有効にしない【検証】([AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.13.2.14-15)。既定のタイマー 64 s は 85 s に近いが、無効なので候補から外す。ただし既定値は EFUSE で変わり得るので、実機で読んで確かめる価値はある。

| 順位 | 仮説 | 支持する証拠 | 反する証拠・未確認点 | REG21 の予想 | 決め手のテスト |
|---|---|---|---|---|---|
| 1 | **二電源の競合**: DIN 5.10 V と Mac の VBUS が U19→VBUS→U18 で並列になり、時間とともに何かが崩れる（ME1502 の逆方向通電での発熱・過温サイクル、Mac ポート側の保護、VBUS の揺れ）。最後は VSYS < VOFF で PMIC が落ちる | 並列構造は【検証】。不具合は両電源接続時だけで、USB のみでは 15 分以上正常。USB と AC アダプタの同時接続で「赤 LED が点滅し動作が安定しない」という CoreS3 の報告がある ([logikara](https://logikara.blog/core-s3-init/))。ME1502 の過温保護は 155 ℃ で自動復帰を繰り返す【検証】ので、レールがゆっくり揺れる説明になり得る | 85 s を生む具体的な仕組みは見つかっていない。SY8303 は軽負荷モードで電流を吸い込まない可能性が高い【推定】。循環電流の大きさは未測定 | b3（VSYS 低電圧）が最有力。VBUS 側の崩れ方によっては b4 | VBUS を切ったデータ専用ケーブルで再現するか。電流の向きが測れる USB-C メーター |
| 2 | **バッテリーなしの VBAT ノード**: 12V があると TP4057（500 mA CC / 4.24 V）が、容量約 10 µF の `BUS_BAT` を介して AXP2101 の BAT ピンを駆動する。AXP2101 側の充電器も同じノードにつながっていて、充電器が 2 台並列になる。電池検出が出たり消えたりし、TP4057 が熱で電流を絞る | 結線は【検証】([DIN 回路図](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf))。TP4057 は電池がないとき CHRG が点滅し STDBY が点灯する【検証】([TP4057 datasheet](https://www.ickey.cn/static-pf/datasheet/b4/69/4fd6/b4694fd6c0e2ac77bd63bcba19d56ef2.pdf))。これは赤 LED の揺らぎと合う。継続的に吸われると約 0.7 W の発熱になり、数十秒で 135 ℃ の電流制限に入り得る【推定】 | 12V のみの長時間試験をまだしていないので、否定も肯定もできない。VBUS がある間、VSYS は通常 VBUS から給電されるので、VBAT の揺れが電源断につながる経路ははっきりしない | b3 か、要因なし | 12V のみで 15 分以上流す。緑 LED の状態。VBAT ノードの電圧。バッテリーを戻す |
| 3 | **フラッシュ済みファームが古い**: M5Unified 0.2.24 より前のファームでは DCDC 低電圧ラッチが有効なまま。どの仮説で起きる小さな落ち込みでも、PMIC がラッチ OFF する | バッテリーなしの CoreS3 SE で REG21=0x20 のラッチ OFF が確認され、電源キーで復帰した ([M5Unified PR #381](https://github.com/m5stack/M5Unified/pull/381))。`^0.2.5` は浮動指定で、libdeps は 2026-10-13 付近に解決されていた。プロジェクトの作業メモでは、2026-10-25 時点で基板に古い moter-controler ファームが載っていた（Issue の観察時のビルドは未確認） | PR #381 の事例は `begin()` の直後に起きたもので、85 s 後ではない。現在の libdeps でビルドすれば無効になる | b5 | 起動時に REG23 を読む（bit0-4 が 0 なら無効化済み） |
| 4 | **入力の過渡と GND ループ**: 24V やリレーの切替、USB の活線挿入で VBUS が瞬間的に 7 V を超える。あるいは Mac・12V アダプタ・24V 系の GND ループ | 24V・リレー切替時や USB 挿入時に落ちたという過去の観察と合う。モーターやリレーの切替は ESP32 リセットの典型的な原因である ([pcbvault](https://pcbvault.com/electronics-troubleshooting/esp32-keeps-resetting.html))。VBUS > 7 V で電源断することは【検証】 | 外部で何も起きていない 85 s 後の電源断は説明できない | b4（まれに b6） | USB 挿入での再現。外部系を外す。USB アイソレータ |
| 5 | **AXP2101 のダイ過温**（145 ℃） | 秒〜分の時定数を持つ唯一の内部要因 | 循環電流は AXP2101 の内部を通らない【推定】。発熱源がはっきりしない | b7 | TDIE（REG 0x3C）と REG00 b1 を 1 Hz で記録する |
| 6 | **DCDC 過電圧**: 外部 5V 系のエンコーダやレベルシフタから 3.3V へ逆流する | OVP（REG23 b5）は有効のまま【検証】 | 根拠は弱い。本体だけの最小構成で再現するなら消える | b6 | 外部系を外す |
| 7 | **ESP32 のブラウンアウト・パニック・ハング** | — | 自動で再起動する仕組みであり、「暗いまま戻らない」と合わない。ただし BOD の後に PMIC が落ちる連鎖は否定できない | 0（または b3 との連鎖） | `esp_reset_reason()` と RTC_NOINIT の生存 |
| 8 | **Mac が USB ポートを止めた** | — | 12V で給電されているので、ポートが止まってもコアの画面は消えないはず | 0 | Mac ログで `CC] active: NO` が先に出るか |
| 9 | **DIN Base の降圧の保護動作** | — | 赤 LED が点いたまま（= DIN 5V は出ている）。SY8303 は 3 A 定格で自動復帰する【検証】([SY8303A datasheet](https://static.chipdip.ru/lib/484/DOC043484845.pdf)) | — | 事実上否定済み |

順位 1 と 2 は排他ではない。どちらも、DIN Base を**バッテリーを抜いた状態**で使っていることが前提になっている。DIN Base は電池を載せたまま使う前提の設計と見られ【推定】、電池があれば VBAT が電池に保持され、AXP2101 の supplement mode が VSYS の落ち込みを支える【検証】([AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.5.3.2)。今回の使い方は、公式に記載のない運用状態である【推定】。順位 3 は、それ自体が引き金ではなく、ほかの仮説で起きる小さな落ち込みを「戻らない電源断」に変える増幅器として働く。いちばん安く消せる仮説なので、最初に潰す。CoreS3 と DIN Base の組み合わせでの電源断の報告は、英語・日本語・中国語のどれでも見つからなかった。近い事例はどれも起動直後か数秒以内に起きており、85 s という遅れ方は前例のない部分である ([M5Stack forum 7288](https://community.m5stack.com/topic/7288/power-not-working-between-cores3-or-cores3-se-lan-module-with-poe-or-rs232-module)、[arcana-mfg](https://arcana-mfg.com/blog/2025/02/07/m5-stack-cores3-se-x-base-aaa%E3%81%A7%E5%9B%B0%E3%81%A3%E3%81%9F%E8%A9%B1/))。

赤 LED の揺らぎは、二つの読み方で仮説 1 と 2 を分ける手がかりになる。データシートによれば、電池がないときは「赤（CHRG）が点滅し、緑（STDBY）が点灯」する。緑が点いていれば TP4057 の教科書どおりの無電池動作で、揺らぎは異常ではない。緑が消えていて赤の明るさだけが揺れているなら、何か（おそらく AXP2101）が `BUS_BAT` から電流を吸い続けているか、LED の電源である `VCC_5V`（= 競合している 5V レールそのもの）が動いている【推定】([TP4057 datasheet](https://www.ickey.cn/static-pf/datasheet/b4/69/4fd6/b4694fd6c0e2ac77bd63bcba19d56ef2.pdf))。Issue には赤 LED のことしか書かれていないので、緑の状態をまず見ておきたい。

## 次の故障では、ケーブルに触る前に電源キーを押す

REG 0x21 は POR でしか消えない。これまでの「両方抜いて 20 s」という復帰手順は、毎回この証拠を消していた。電源キー（S2 → AXP2101 の PWRON）は OFF 状態からの起動要因なので【検証】([SE 回路図 p1](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_01.png)、[AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.5.4.2)、キーで起こせば POR なしで要因を読める。PR #381 の事例でも、ラッチは電源キーで解けている【検証】([PR #381](https://github.com/m5stack/M5Unified/pull/381))。

ただし前提が二つある。一つ目に、**キーで起動したファームが、何かを書き込む前に REG 0x20/0x21 を記録しなければならない**。いまのファームは、起こしても何も残さない。二つ目に、REG 0x21 は前回の POR 以降の要因を OR で積み上げる【推定、Reset=POR から】。そのため試験の前に一度、両方の電源を抜いて POR をかけ、起動時に REG21=0 であることを確かめてから始める。M5Unified の `begin()` は 0x20、0x21 も IRQ 状態も触らないので、`M5.begin()` の直後に `M5.Power.Axp2101.readRegister8(0x21)` で読める【検証】([AXP2101_Class.hpp](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/power/AXP2101_Class.hpp))。

記録用ファームに入れる内容は次のとおり（未コンパイル。診断ノートの雛形を縮めたもの）。起動時には、`esp_reset_reason()`、REG 0x20、0x21、0x48-0x4A（IRQ）、0x00、0x01 を記録する。あわせて EFUSE 依存の基準値として 0x22、0x23、0x24、0x27、0x15、0x16、0x18、0x19、0x68 を読み、**LCD とシリアルと NVS** に残す。USB が切れた瞬間を見たいのだから、シリアルだけに頼ってはいけない。動作中は 1 s ごとに、`getVBUSVoltage()`、`getAPSVoltage()`（VSYS）、`getTSVoltage()×2`（≒ `BUS_OUT`）、`getBatteryVoltage()`（VBAT ノード）、`getInternalTemperature()`、REG00 の b0（限流中）、b1（熱制御中）、b3（電池あり）、b5（VBUS good）、REG01 の b3（VINDPM）と充電状態を記録する。稼働秒数は AXP2101 の DATA_BUFFER（0x04-0x07。PMIC の OFF では残り、POR で消える）と NVS にも書く【検証】([AXP2101 DS V1.4](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf) §6.13.2.3-6)。注意点として、`getVBUSVoltage()` は REG00 b5 が 0 のとき 0.0 を返す。0 が出ても 0 V とは限らない。

```cpp
// M5.begin(cfg) の直後（例）
auto& a = M5.Power.Axp2101;
uint8_t on = a.readRegister8(0x20), off = a.readRegister8(0x21), r23 = a.readRegister8(0x23);
M5.Display.printf("RST=%d ON=%02X OFF=%02X R23=%02X\n", (int)esp_reset_reason(), on, off, r23);
// 同じ値を Serial と Preferences(NVS) にも残す。0x48-0x4A は読んだ後に同じ値を書き戻してクリアする（RW1C）
```

故障時は、次の順で進める。

| 順 | 操作（電源はまだ抜かない） | 読み方 |
|---|---|---|
| a | 観察する: 緑 LED の点灯の有無、赤 LED が点滅か連続点灯か。テスターがあれば Port A（赤 Grove）の 5V ピン（= `BUS_OUT`）と、DIN Base の電池コネクタ P1 の BAT+（= `BUS_BAT`、S1 が ON のとき）を GND に対して測る | `BUS_OUT` が約 5 V なら、VBUS が DIN に保持されている（ラッチ機構の裏付け）。VBAT が 4.2 V 付近か、揺れているか（仮説 2） |
| b | Mac で `/usr/bin/log show --last 10m --style compact --predicate 'process == "kernel"' \| grep -E 'terminateDevice\|CC\] active\|cableChange\|over-?current'` を実行する | `terminateDevice … hardware connection lost` の前に `CC] active: NO` が**なければ**、デバイス側の電源断。あればケーブル・コネクタ・ポート側の事象（仮説 8） |
| c | 両方のケーブルを挿したまま、電源キーを 1-2 s 押す | 起動したら、PMIC の OFF 状態で確定。REG21 を読む。b3 = 入力の落ち込みや競合（仮説 1/2）、b4 = 過渡（仮説 4）、b5 = 古いファーム（仮説 3）、b6 = 逆流（仮説 6）、b7 = 過温（仮説 5）、b1 = ソフト電源断（ファームの不具合） |
| d | c で起動しなければ、USB だけを抜いてキーを押す。だめなら USB を戻して 12V だけを抜き、キーを押す | どの組み合わせで起動したかを記録する。両電源がある状態で起動しないなら、AXP2101 が入力を「good」と見ていない（REG00 b5 の判定。§6.5.3.1）可能性があり、問題は上流の 5V 源の状態にある【推定】 |
| e | それでも戻らなければ、初めて両方を抜く | REG21 の証拠は消える。復帰までの秒数を測る |

キーの ONLEVEL は、M5Unified が書く 0x27=0x00 ではデータシート上 128 ms になる。M5Unified のコメントにある「Hold=1sec」は IRQLEVEL を指すと見られるが、ONLEVEL を指すかどうかは未確認である。1-2 s 押せばどちらの解釈でも足りる。4 s 以上押し続ける必要はない。85 s を待つ代わりに、「12V を入れた状態で USB を挿す」という既知の引き金を使えば数秒で再現できる。最初の REG21 は、この方法で取るのが早い。

## 安い順の切り分け計画

各段は、前の段の結果で次に何をするかが変わる。費用はどれも小さく、最も高いもので USB アイソレータ基板の購入程度である（表中の金額は目安）。

| 段 | 試験（費用） | 方法 | 確定・否定できる仮説 |
|---|---|---|---|
| 0 | 記録用ファームの書き込みと基準値の記録（無料） | 前節の内容。一度 POR をかけて REG21=0 を確認してから始める | REG23 bit0-4 ≠ 0 なら**仮説 3 が確定**（いまの libdeps で作り直すだけで挙動が変わる）。EFUSE の実値（VOFF、ウォッチドッグ、電池検出）を確定する |
| 1 | USB 挿入による即時再現と、キーでの復帰（無料） | 12V 給電中に USB を挿し、落ちたら前節の a-e の手順を行う | REG21 の要因ビットで、仮説 1-6 のどれかを名指しできる。キーで起動すれば仮説 7、8 は否定される |
| 2 | 12V のみで 15 分以上流す（無料） | USB なし、同じ負荷で。ログは LCD と NVS で確認する | 落ちたら仮説 1 と 8 は否定され、**仮説 2（VBAT）か DIN 単独の問題**になる。落ちなければ、2 系統目の 5V か USB の GND が関わっている |
| 3 | 12V＋USB を VBUS カットのアダプタ経由で接続する（数百円） | CC、D+/D−、GND は通し、VBUS だけを切る。12 Mbps で列挙されることを確認する | 落ちなければ**仮説 1 が確定**。落ちるなら 5V の競合は不要で、仮説 2、4 か GND が残る。ESP32-S3 の USB-Serial-JTAG は電源投入時から D+ をプルアップしているので、VBUS がなくても列挙できる見込みである【推定】([esp-iot-solution](https://docs.espressif.com/projects/esp-iot-solution/en/latest/usb/usb_overview/usb_serial_jtag.html)) |
| 4 | 計測器を入れる（数千円） | 電流の向きが測れる USB-C メーター（例: POWER-Z KM003C [ChargerLAB](https://www.chargerlab.com/bi-directional-current-detection-the-cool-feature-of-chargerlab-power-z-km003c)）を Mac 側に入れ、12V の有無で IBUS の符号と VBUS を比べる。符号の向きは文書化されていないので、先に USB のみの状態で校正する。テスターの min/max で Port A の 5V と 12V を 85 s 間記録する。USB を抜いた状態で CoreS3 の USB-C VBUS ピンの電圧を見る。85 s 前後で U18/U19 付近、TP4057、SY8303 の温度を指や放射温度計で確かめる | IBUS が反転するか 0 に近づけば、**Mac への逆流（仮説 1）**。5V が 4.36 V（VINDPM 既定値）に向かって下がっていく、電流が増える、温度が上がる、のどれかがあれば熱か過電流。TP4057 だけが熱ければ仮説 2 |
| 5 | バッテリーを戻す（無料） | DIN Base に 500 mAh の電池を戻し、両電源をつないで 15 分以上流す | 落ちなくなれば、**バッテリーなし運用が原因**（仮説 2、または電池の緩衝がないことで仮説 1 が顕在化）【推定】。運用上の回避策にもなる |
| 6 | 外部系を外す（無料） | CoreS3、DIN、12V、USB だけの最小構成。再現したら一つずつ戻す（GND → 5V エンコーダとレベルシフタ → 24V 待機 → リレー切替） | 最小構成で 85 s の電源断が出れば、仮説 4、6 は 85 s の症状については否定される。戻したときに出たものが原因 |
| 7 | USB アイソレータ（ADuM3160 系、数千円） | Mac と CoreS3 の間に入れる。ただし絶縁側の 5V を VBUS に出す基板が多いので、段 3 の VBUS カットと組み合わせる ([wagiminator](https://github.com/wagiminator/ADuM3160-USB-Isolator)) | 段 3 で落ち、段 7 で落ちなければ、**Mac 経由の GND ループ（仮説 4）**。「2 電源」と「GND」の変数を分けられる |
| 常時 | Mac ログの常時取得（無料） | `/usr/bin/log stream --style compact --predicate 'process == "kernel" AND (senderImagePath CONTAINS[c] "IOUSBHostFamily" OR senderImagePath CONTAINS[c] "IOAccessoryManager") AND NOT (eventMessage CONTAINS "AppleUSBIORequest::complete")'`。zsh では `log` が組み込みコマンドとぶつかるので、フルパスで呼ぶ | デバイス側の電源断か、ポート側の事象かを切り分ける（仮説 8） |

この Mac のログには、2026-10-25 21:51 と 22:49 に CoreS3 の `terminateDevice` が残っている。どちらも約 70 ms 前に `CC] active: NO` が出ているので、物理的な抜き差しに見える。また 20:43-21:51 には、bulk IN の transaction error が毎分約 2 万行の勢いで続いていた（ローカルの観察）。これが Issue #1 の故障時刻と重なるかどうかは、作業メモと照合するまで判断しない。

当面の回避策は、仮説 1 が当たっていてもいなくても有効なものを選ぶ。開発中に 12V と USB を同時につなぐ必要があるなら、VBUS カットのアダプタ（段 3）を常用する。これで 5V の並列は物理的になくなる。USB だけで動かす既存の `*-usbpower` ビルドは、競合のない構成として引き続き使える。ファームの設定で二つの 5V を切り離す方法はない。それができると考えて `output_power` を書き換えても、取り消されるか、`BUS_OUT` 上の衝突の形が変わるだけである。

## 結論

今回の調査で、問題の性質が変わった。「CoreS3 が不定期に落ちる」という一つの謎は、「AXP2101 が何かで OFF になると、DIN の 5V がその状態を固定してしまう」という、**回路図で説明できる仕組み**と、「その何かが 85 s で起きる」という**未解明の引き金**に分かれた。前者は、両電源を並列にする CoreS3 の電源経路と、バッテリーを外した運用が重なって生まれたものである。後者を推測で詰めるより、REG 0x21 に答えさせる方が早く確実である。これまでの復帰手順は、毎回その答えを消していた。

次の一手は二つで済む。記録用のファームを書き、次に落ちたら電源を抜く前にキーを押すこと。そして 12V のみの長時間試験と VBUS カットでの試験で、「2 系統目の 5V が必要か」を確かめることである。REG23 で古いファームの可能性を消し、REG21 のビットと二つの試験の結果がそろえば、表の仮説のほとんどは 1 日以内の作業で確定するか否定される。残る可能性が高いのは、85 s という時間の物理的な由来である。熱か、レールの揺れか、充電器の振動か。これだけは、温度と電圧の 1 Hz ログを見るまで決められない。
