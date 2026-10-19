# cmiv2029 プロジェクト

CMI200、300シリーズ照明コントローラソフトウェアです。AVRマイコンとLUFA（Lightweight USB Framework for AVRs）ライブラリを使用した、USBデバイス（Virtual Serial / 仮想シリアルポート等）のファームウェア開発プロジェクトです。I2CやSPIによる周辺機器制御の実装も含まれています。

## 📁 ディレクトリ構成

*   `.vscode/` - VS Code用の設定フォルダ
*   `Config/` - アプリケーションやLUFAの設定ファイル
*   `LUFA/` - LUFAフレームワーク本体（またはサブモジュール）
*   `Descriptors.c / .h` - USB記述子（ディスクリプタ）定義
*   `main.c` - メイン処理ルーチン
*   `i2c.c / .h` - I2C通信制御用ソース
*   `spi.c / .h` - SPI通信制御用ソース
*   `Serial.c / .h` / `VirtualSerial.h` - シリアル通信処理用ソース
*   `Makefile` - ビルド用設定ファイル

## 🛠️ ビルド方法

### 前提条件
*   **AVR GCC Toolchain** (avr-gcc, avr-libc など)
*   **Make** ビルドツール

### コマンド
プロジェクトのルートディレクトリで以下のコマンドを実行してビルドします。

```bash
make
```

ビルドが成功すると、以下のオブジェクトファイルや書き込み用バイナリが生成されます。
*   `cmiv2029.hex` (Intel HEX形式)
*   `cmiv2029.bin` (バイナリ形式)
*   `cmiv2029.elf` (ELF形式)
*   `cmiv2029.eep` (EEPROMデータ)

ファイルをクリーンアップ（削除）する場合は以下を実行します。

```bash
make clean
```

## 🚀 書き込み方法

環境に合わせて `avrdude` や各種プログラマ（dfu-programmer など）を使用して、生成された `cmiv2029.hex` または `cmiv2029.bin` をターゲットのAVRマイコンに書き込んでください。

## 📄 ライセンス
このプロジェクトのライセンスは `LICENSE` ファイルを参照してください。
