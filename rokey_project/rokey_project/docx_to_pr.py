from docx import Document
import json
import qrcode

def parse_prescription_from_docx(docx_path):
    doc = Document(docx_path)
    tables = doc.tables
    data = {
        "교부일자": "",
        "의료기관": "",
        "전화번호": "",
        "환자": {
            "성명": "",
            "주민등록번호": ""
       },
        "처방의약품": [],
        "참고사항": ""
    }
    
    # 표 순회
    for table in tables:
        for row in table.rows:
            cells = [cell.text.strip() for cell in row.cells]

            # 성명
            if any("성명" in c for c in cells):
                for i, c in enumerate(cells):
                    if "성명" in c and i + 1 < len(cells):
                        data["환자"]["성명"] = cells[i + 1]

            # 주민등록번호
            if any("주민등록번호" in c for c in cells):
                for i, c in enumerate(cells):
                    if "주민등록번호" in c and i + 1 < len(cells):
                        data["환자"]["주민등록번호"] = cells[i + 1]

            # 의료기관명
            if any("명칭" in c or "명칭" in c for c in cells):
                for i, c in enumerate(cells):
                    if "명칭" in c or "명칭" in c and i + 1 < len(cells):
                        data["의료기관"] = cells[i + 1]
            
            # 전화번호
            if any("전화번호" in c or "전화번호" in c for c in cells):
                for i, c in enumerate(cells):
                    if "전화번호" in c or "전화번호" in c and i + 1 < len(cells):
                        data["전화번호"] = cells[i + 1]

            # 교부일자
            if any("교부 연월일" in c for c in cells):
                for c in cells:
                    if "년" in c and "월" in c and "일" in c:
                        data["교부일자"] = c.replace(" ", "").replace("년", "-").replace("월", "-").replace("일", "")
    
    # 참고사항: 별도 루프로 아래 행에서 추출
    for table in tables:
        for row_idx, row in enumerate(table.rows):
            cells = [cell.text.strip() for cell in row.cells]
            for i, c in enumerate(cells):
                if "참고사항" in c:
                    # 다음 행 존재할 경우
                    if row_idx + 1 < len(table.rows):
                        next_row = table.rows[row_idx + 1]
                        if i < len(next_row.cells):  # 열 범위 확인
                            value = next_row.cells[i].text.strip()
                            if value:
                                data["참고사항"] = value

                       
                if "처방 의약품의 명칭" in cells[0]:
                    # 다음 행부터 약품 데이터 시작
                    for next_row in table.rows[row_idx + 1:]:
                        next_cells = [cell.text.strip() for cell in next_row.cells]
                        #print(next_cells)
                        # 첫 번째 셀이 비어있으면 종료
                        if not next_cells[0]:
                            break

                        # 필요한 열이 4개 이상 있는지 확인
                        if len(next_cells) >= 4:
                            data["처방의약품"].append({
                                "이름": next_cells[0],
                                "1회 투약량": next_cells[9],
                                "1일 투여횟수": next_cells[10],
                                "총 투약일수": next_cells[14]
                            })
                    break  # 다른 테이블에서 중복 파싱 방지
    return {"처방전": data}

def generate_qr(data, output_file="/home/hongha/smart_pharmacist_ws/src/rokey_project/image/처방전_QR.png"):
    qr_data = json.dumps(data, ensure_ascii=False)
    img = qrcode.make(qr_data)
    img.save(output_file)
    print(f"✅ QR 코드 저장 위치: {output_file}")

# 실행 예시
docx_path = "/home/hongha/smart_pharmacist_ws/src/rokey_project/image/처방전.docx"  # 정확한 경로로 수정
parsed = parse_prescription_from_docx(docx_path)
generate_qr(parsed)
print(json.dumps(parsed, indent=2, ensure_ascii=False))
