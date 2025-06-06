'''처방전 파일의 내용을 QR코드로 생성해주는 파일'''

from docx import Document
import qrcode

def parse_prescription_from_docx(docx_path):
    doc = Document(docx_path)
    tables = doc.tables
    name = ""
    id_number = ""
    medicines = []

    for table in tables:
        for row_idx, row in enumerate(table.rows):
            cells = [cell.text.strip() for cell in row.cells]

            # 성명
            if any("성명" in c for c in cells):
                for i, c in enumerate(cells):
                    if "성명" in c and i + 1 < len(cells):
                        name = cells[i + 1]

            # 주민등록번호
            if any("주민등록번호" in c for c in cells):
                for i, c in enumerate(cells):
                    if "주민등록번호" in c and i + 1 < len(cells):
                        id_number = cells[i + 1]

            # 처방 의약품
            if "처방 의약품의 명칭" in cells[0]:
                for next_row in table.rows[row_idx + 1:]:
                    next_cells = [cell.text.strip() for cell in next_row.cells]
                    if not next_cells[0]:  # 첫 열이 비어 있으면 종료
                        break
                    if len(next_cells) >= 4:
                        name_code = next_cells[0].split()[0]  # 예: A02X nexilen_tab → A02X
                        dose = next_cells[9]
                        times = next_cells[10]
                        days = next_cells[14]
                        medicines.append(f"{name_code} {dose} {times} {days}")
                break  # 중복 방지

    # 최종 출력 문자열
    result = f"{name} {id_number}\n" + "\n".join(medicines)
    return result

def generate_qr(text, output_file="/home/hongha/rokey_pharmacy_ws/src/rokey_project/image/QR_code/처방전_QR_dyspepsia.png"):
    img = qrcode.make(text)
    img.save(output_file)
    print(f"✅ QR 코드 저장 위치: {output_file}")

# 실행 예시
docx_path = "/home/hongha/rokey_pharmacy_ws/src/rokey_project/image/처방전_dyspepsia.docx"
qr_text = parse_prescription_from_docx(docx_path)
generate_qr(qr_text)
print(qr_text)
