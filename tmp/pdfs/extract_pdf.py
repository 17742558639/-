import sys

import pdfplumber


path = sys.argv[1]
keywords = [
    "技术要求",
    "比赛流程",
    "任务流程",
    "竞赛流程",
    "启停区",
    "二维码",
    "原料区",
    "粗加工区",
    "暂存区",
    "场地",
    "坐标",
    "机械臂",
    "自主移动",
    "巡检",
]

with pdfplumber.open(path) as pdf:
    print(f"PAGES {len(pdf.pages)}")
    for page_number, page in enumerate(pdf.pages, 1):
        text = page.extract_text() or ""
        hits = [keyword for keyword in keywords if keyword in text]
        if hits:
            print(f"PAGE {page_number} HITS {','.join(hits)}")
            print(text[:5000].replace("\x00", " "))
            print("---")
