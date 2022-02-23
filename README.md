# urarchallenge_analyzer
ビデオファイルとCSVファイルを使って対象ウマ娘が馬か人間かを判定するプログラム（ROS2使用）

## 対象

- 640x360の動画
- ROS2で動く物体検出プログラム（返り値：bbox_ex_msgsに基づく座標と検出データ）

## モデル

[YOLOv5](YOLOv5/README.md)

## 検出手順

検出位置を記録したCSVファイルについて：`urarachallenge-export-CSV`を用いて動画に対応したCSVファイルを生成してください。（urarachallenge-export-CSVのREADMEはこのフォルダ内にあります。）

![](https://raw.githubusercontent.com/Ar-Ray-code/urara-challenge/main/images_for_readme/1st-result/how2detect.jpeg)
