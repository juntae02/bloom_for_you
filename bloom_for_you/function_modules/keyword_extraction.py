import os
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain_community.chat_models import ChatOpenAI
from stt import stt
import warnings

from langchain.prompts import PromptTemplate

package_path = "/home/rokey/r2_ws/src/bloom_for_you"


def _get_prompt(file_name: str):
    # 1. 절대경로일 경우: 그대로 사용
    if os.path.isabs(file_name):
        file_path = file_name
    else:
    # 2. 상대경로일 경우: 패키지의 resource 디렉토리 기준으로 찾음
        resource_path = os.path.join(package_path, "resource")
        file_path = os.path.join(resource_path, file_name)
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

def keyword_extraction(openai_api_key=None):
    if openai_api_key is None:
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key is None:
            raise ValueError("API 키를 인자로 전달하거나 환경변수에 설정하세요.")

    llm = ChatOpenAI(
        model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
    )
    prompt_content = _get_prompt("test.txt")

    prompt_template = PromptTemplate(
        input_variables=["user_input"], template=prompt_content
    )
    lang_chain = LLMChain(llm=llm, prompt=prompt_template)

    output_message = stt(openai_api_key)
    response = lang_chain.invoke({"user_input": output_message})
   
    print(f"반환할 대답: {response}")
    return response


if __name__ == "__main__":
    keyword_extraction()
