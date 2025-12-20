package main

import (
	"bufio"
	"fmt"
	"os"
	"regexp"
	"strconv"
	"strings"
)

// sr_96k形式の32ビット値を sr_48k形式のバイト配列に変換するスクリプト
// 例: {0x00002080} -> {0x00, 0x00, 0x20, 0x80}
// 例: {0x00FA2F0A, 0xFE010ED5, ...} -> {0x00, 0xFA, 0x2F, 0x0A, 0xFE, 0x01, 0x0E, 0xD5, ...}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("使用方法: go run convert_96k_to_48k.go <入力ファイル> [出力ファイル]")
		fmt.Println("例: go run convert_96k_to_48k.go sr_96k_Modes.h sr_48k_Modes_converted.h")
		os.Exit(1)
	}

	inputFile := os.Args[1]
	outputFile := "output.h"
	if len(os.Args) >= 3 {
		outputFile = os.Args[2]
	}

	err := convertFile(inputFile, outputFile)
	if err != nil {
		fmt.Printf("エラー: %v\n", err)
		os.Exit(1)
	}

	fmt.Printf("変換完了: %s -> %s\n", inputFile, outputFile)
}

func convertFile(inputPath, outputPath string) error {
	file, err := os.Open(inputPath)
	if err != nil {
		return fmt.Errorf("入力ファイルを開けません: %w", err)
	}
	defer file.Close()

	var lines []string
	scanner := bufio.NewScanner(file)
	// 長い行を処理するためにバッファサイズを増やす
	buf := make([]byte, 0, 64*1024)
	scanner.Buffer(buf, 1024*1024)

	for scanner.Scan() {
		lines = append(lines, scanner.Text())
	}

	if err := scanner.Err(); err != nil {
		return fmt.Errorf("ファイル読み込みエラー: %w", err)
	}

	convertedLines := convertLines(lines)

	outFile, err := os.Create(outputPath)
	if err != nil {
		return fmt.Errorf("出力ファイルを作成できません: %w", err)
	}
	defer outFile.Close()

	writer := bufio.NewWriter(outFile)
	for _, line := range convertedLines {
		_, err := writer.WriteString(line + "\n")
		if err != nil {
			return fmt.Errorf("書き込みエラー: %w", err)
		}
	}

	return writer.Flush()
}

func convertLines(lines []string) []string {
	var result []string

	// 配列定義の正規表現
	// 例: ADI_REG_TYPE sr_96k_0[1] = {0x00002080};
	arrayDefRegex := regexp.MustCompile(`^(ADI_REG_TYPE\s+)(\w+)(\[)(\d+)(\]\s*=\s*\{)(.+?)(\};?)(.*)$`)

	// 複数行にまたがる配列定義の開始
	arrayStartRegex := regexp.MustCompile(`^(ADI_REG_TYPE\s+)(\w+)(\[)(\d+)(\]\s*=\s*\{)(.*)$`)

	// SIGMA_WRITE_REGISTER_BLOCK の正規表現
	sigmaWriteRegex := regexp.MustCompile(`^(SIGMA_WRITE_REGISTER_BLOCK\(\s*\w+,\s*0x[0-9A-Fa-f]+,\s*)(\d+)(,\s*)(\w+)(.*)$`)

	i := 0
	for i < len(lines) {
		line := lines[i]

		// 単一行の配列定義
		if matches := arrayDefRegex.FindStringSubmatch(line); matches != nil {
			converted := convertArrayDefinition(matches)
			result = append(result, converted)
			i++
			continue
		}

		// 複数行にまたがる配列定義
		if matches := arrayStartRegex.FindStringSubmatch(line); matches != nil && !strings.HasSuffix(strings.TrimSpace(line), ";") {
			// 配列が複数行にまたがっている場合、全体を読み込む
			fullLine := line
			i++
			for i < len(lines) && !strings.Contains(fullLine, "};") {
				fullLine += " " + strings.TrimSpace(lines[i])
				i++
			}

			// 結合した行を変換
			if matches := arrayDefRegex.FindStringSubmatch(fullLine); matches != nil {
				converted := convertArrayDefinition(matches)
				result = append(result, converted)
			} else {
				// 変換できない場合はそのまま出力
				result = append(result, fullLine)
			}
			continue
		}

		// SIGMA_WRITE_REGISTER_BLOCK の変換
		if matches := sigmaWriteRegex.FindStringSubmatch(line); matches != nil {
			converted := convertSigmaWrite(matches)
			result = append(result, converted)
			i++
			continue
		}

		// その他の行はそのまま出力（変数名・関数名を変えない）
		result = append(result, line)
		i++
	}

	return result
}

func convertArrayDefinition(matches []string) string {
	// matches[1]: "ADI_REG_TYPE "
	// matches[2]: 変数名 (例: "sr_96k_0") - 変更しない
	// matches[3]: "["
	// matches[4]: 配列サイズ
	// matches[5]: "] = {"
	// matches[6]: 配列の中身
	// matches[7]: "};" または "}"
	// matches[8]: 残り（コメントなど）

	varName := matches[2] // 変数名はそのまま
	arrayContent := matches[6]

	// 32ビット値を抽出
	hexRegex := regexp.MustCompile(`0x([0-9A-Fa-f]+)`)
	hexMatches := hexRegex.FindAllStringSubmatch(arrayContent, -1)

	var bytes []string
	for _, hexMatch := range hexMatches {
		hexValue := hexMatch[1]
		// 8桁に正規化（32ビット = 8桁の16進数）
		hexValue = fmt.Sprintf("%08s", hexValue)
		if len(hexValue) > 8 {
			hexValue = hexValue[len(hexValue)-8:]
		}

		// 4バイトに分解
		for j := 0; j < 8; j += 2 {
			byteStr := fmt.Sprintf("0x%s", strings.ToUpper(hexValue[j:j+2]))
			bytes = append(bytes, byteStr)
		}
	}

	newArraySize := len(bytes)
	newContent := strings.Join(bytes, ", ")

	return fmt.Sprintf("%s%s%s%d%s%s%s%s",
		matches[1], varName, matches[3], newArraySize, matches[5], newContent, matches[7], matches[8])
}

func convertSigmaWrite(matches []string) string {
	// matches[1]: "SIGMA_WRITE_REGISTER_BLOCK( DEVICE_ADDR_0, 0xXXXX, "
	// matches[2]: バイト数
	// matches[3]: ", "
	// matches[4]: 変数名 - 変更しない
	// matches[5]: 残り

	oldSize, err := strconv.Atoi(matches[2])
	if err != nil {
		return matches[0] // 変換できない場合はそのまま返す
	}

	// ワード数からバイト数に変換 (1ワード = 4バイト)
	newSize := oldSize * 4

	return fmt.Sprintf("%s%d%s%s%s", matches[1], newSize, matches[3], matches[4], matches[5])
}
