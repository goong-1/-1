import sqlite3

def check_data():
    conn = sqlite3.connect('robot_data.db')
    cursor = conn.cursor()
    
    # 최신 데이터 10개 조회
    cursor.execute('SELECT * FROM telemetry ORDER BY timestamp DESC LIMIT 100')
    rows = cursor.fetchall()
    
    print(f"{'ID':<5} | {'Temp':<7} | {'Speed':<7} | {'Timestamp'}")
    print("-" * 45)
    
    for row in rows:
        print(f"{row[0]:<5} | {row[1]:<7} | {row[2]:<7} | {row[3]}")
    
    conn.close()

if __name__ == "__main__":
    check_data()