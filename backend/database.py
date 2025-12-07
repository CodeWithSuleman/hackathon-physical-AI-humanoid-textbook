# backend/database.py

import os
import asyncpg
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

DATABASE_URL = os.getenv("DATABASE_URL")

async def get_connection():
    """
    Establishes and returns a connection to the PostgreSQL database.
    """
    if not DATABASE_URL:
        raise ValueError("DATABASE_URL environment variable is not set.")
    
    conn = await asyncpg.connect(DATABASE_URL)
    return conn

async def close_connection(conn):
    """
    Closes the database connection.
    """
    await conn.close()

async def init_db():
    """
    Initializes the database schema if necessary.
    This function can be expanded to create tables, etc.
    """
    conn = None
    try:
        conn = await get_connection()
        # Example: Create a simple 'modules' table if it doesn't exist
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS modules (
                id VARCHAR(255) PRIMARY KEY,
                title VARCHAR(255) NOT NULL,
                description TEXT
            );
        """)
        print("Database initialized successfully.")
    except Exception as e:
        print(f"Error initializing database: {e}")
    finally:
        if conn:
            await close_connection(conn)

if __name__ == "__main__":
    # Example usage (for testing connection)
    import asyncio

    async def test_connection():
        print("Testing database connection...")
        conn = None
        try:
            conn = await get_connection()
            print("Connection successful!")
            # await init_db() # Uncomment to run schema initialization on test run
        except Exception as e:
            print(f"Connection failed: {e}")
        finally:
            if conn:
                await close_connection(conn)
            print("Connection closed.")

    asyncio.run(test_connection())
